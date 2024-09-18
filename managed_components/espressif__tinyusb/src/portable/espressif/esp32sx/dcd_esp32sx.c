#include "tusb_option.h"

#if (((CFG_TUSB_MCU == OPT_MCU_ESP32S2) ||  (CFG_TUSB_MCU == OPT_MCU_ESP32S3)) && CFG_TUD_ENABLED)

// Espressif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "soc/dport_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/usb_periph.h"
#include "soc/periph_defs.h" // for interrupt source

#include "device/dcd.h"

// Max number of bi-directional endpoints including EP0
#define EP_MAX            USB_OUT_EP_NUM

// FIFO size in bytes
#define EP_FIFO_SIZE      1024

// Max number of IN EP FIFOs
#define EP_FIFO_NUM 5

typedef struct {
    uint8_t *buffer;
    uint16_t total_len;
    uint16_t queued_len;
    uint16_t max_size;
    bool short_packet;
    uint8_t interval;
} xfer_ctl_t;

static const char *TAG = "TUSB:DCD";
static intr_handle_t usb_ih;
static uint32_t _setup_packet[2];

#define XFER_CTL_BASE(_ep, _dir) &xfer_status[_ep][_dir]
static xfer_ctl_t xfer_status[EP_MAX][2];

static uint8_t _allocated_fifos = 1;

// Will either return an unused FIFO number, or 0 if all are used.
static uint8_t get_free_fifo(void)
{
    ESP_EARLY_LOGV(TAG, "Allocating FIFO");
    if (_allocated_fifos < EP_FIFO_NUM) return _allocated_fifos++;
    return 0;
}

// Setup the control endpoint 0.
static void bus_reset(void)
{
    ESP_EARLY_LOGV(TAG, "Bus reset");
    for (int ep_num = 0; ep_num < USB_OUT_EP_NUM; ep_num++) {
        USB0.out_ep_reg[ep_num].doepctl |= USB_DO_SNAK0_M;
    }

    // clear device address
    USB0.dcfg &= ~USB_DEVADDR_M;

    USB0.daintmsk = USB_OUTEPMSK0_M | USB_INEPMSK0_M;
    USB0.doepmsk  = USB_SETUPMSK_M | USB_XFERCOMPLMSK;
    USB0.diepmsk  = USB_TIMEOUTMSK_M | USB_DI_XFERCOMPLMSK_M;

    USB0.grstctl |= 0x10 << USB_TXFNUM_S; // fifo 0x10
    USB0.grstctl |= USB_TXFFLSH_M;
    USB0.grxfsiz = 52;

    USB0.gnptxfsiz = (16 << USB_NPTXFDEP_S) | (USB0.grxfsiz & 0x0000ffffUL);

    USB0.out_ep_reg[0].doeptsiz |= USB_SUPCNT0_M;

    USB0.gintmsk |= USB_IEPINTMSK_M | USB_OEPINTMSK_M;
}

static void enum_done_processing(void)
{
    ESP_EARLY_LOGV(TAG, "Enumeration done");
    uint32_t enum_spd = (USB0.dsts >> USB_ENUMSPD_S) & (USB_ENUMSPD_V);

    if (enum_spd == 0x03) { // Full-Speed (PHY on 48 MHz)
        USB0.in_ep_reg[0].diepctl &= ~USB_D_MPS0_V;    // 64 bytes
        USB0.in_ep_reg[0].diepctl &= ~USB_D_STALL0_M; // clear Stall
        xfer_status[0][TUSB_DIR_OUT].max_size = 64;
        xfer_status[0][TUSB_DIR_IN].max_size = 64;
    } else {
        USB0.in_ep_reg[0].diepctl |= USB_D_MPS0_V;     // 8 bytes
        USB0.in_ep_reg[0].diepctl &= ~USB_D_STALL0_M; // clear Stall
        xfer_status[0][TUSB_DIR_OUT].max_size = 8;
        xfer_status[0][TUSB_DIR_IN].max_size = 8;
    }
}

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
void dcd_init(uint8_t rhport)
{
    ESP_EARLY_LOGV(TAG, "DCD init");
    USB0.dctl |= USB_SFTDISCON_M;

    USB0.dcfg |= USB_NZSTSOUTHSHK_M | (3 << 0);

    USB0.gahbcfg |= USB_NPTXFEMPLVL_M | USB_GLBLLNTRMSK_M;
    USB0.gusbcfg |= USB_FORCEDEVMODE_M;
    USB0.gotgctl &= ~(USB_BVALIDOVVAL_M | USB_BVALIDOVEN_M | USB_VBVALIDOVVAL_M);

    for (int n = 0; n < USB_OUT_EP_NUM; n++) {
        USB0.out_ep_reg[n].doepctl |= USB_DO_SNAK0_M;
    }

    USB0.gintmsk = 0;
    USB0.gotgint = ~0U;
    USB0.gintsts = ~0U;
    USB0.gintmsk = USB_OTGINTMSK_M | USB_MODEMISMSK_M | USB_RXFLVIMSK_M | USB_ERLYSUSPMSK_M |
                   USB_USBSUSPMSK_M | USB_WKUPINTMSK_M | USB_USBRSTMSK_M |
                   USB_ENUMDONEMSK_M | USB_RESETDETMSK_M | USB_DISCONNINTMSK_M;

    dcd_connect(rhport);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    ESP_EARLY_LOGV(TAG, "Set address: %u", dev_addr);
    USB0.dcfg |= ((dev_addr & USB_DEVADDR_V) << USB_DEVADDR_S);
    dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
    ESP_EARLY_LOGV(TAG, "Remote wakeup");
    USB0.dctl |= USB_RMTWKUPSIG_M;

    USB0.gintsts = USB_SOF_M;
    USB0.gintmsk |= USB_SOFMSK_M;

    vTaskDelay(pdMS_TO_TICKS(1));

    USB0.dctl &= ~USB_RMTWKUPSIG_M;
}

void dcd_connect(uint8_t rhport)
{
    ESP_EARLY_LOGV(TAG, "Connect");
    USB0.dctl &= ~USB_SFTDISCON_M;
}

void dcd_disconnect(uint8_t rhport)
{
    ESP_EARLY_LOGV(TAG, "Disconnect");
    USB0.dctl |= USB_SFTDISCON_M;
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_edpt)
{
    ESP_EARLY_LOGV(TAG, "Endpoint opened");
    (void)rhport;
    usb_out_endpoint_t *out_ep = &(USB0.out_ep_reg[0]);
    usb_in_endpoint_t *in_ep = &(USB0.in_ep_reg[0]);

    uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
    uint8_t const dir = tu_edpt_dir(desc_edpt->bEndpointAddress);

    xfer_ctl_t *xfer = XFER_CTL_BASE(epnum, dir);
    xfer->max_size = tu_edpt_packet_size(desc_edpt);
    xfer->interval = desc_edpt->bInterval;

    if (dir == TUSB_DIR_OUT) {
        ESP_EARLY_LOGV(TAG, "OUT Endpoint %d", epnum);
        out_ep[epnum].doepctl |= USB_USBACTEP1_M |
                                 desc_edpt->bmAttributes.xfer << USB_EPTYPE1_S |
                                 (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS ? USB_DO_SETD0PID1_M : 0) |
                                 xfer->max_size << USB_MPS1_S;
        USB0.daintmsk |= (1 << (16 + epnum));
    } else {
        ESP_EARLY_LOGV(TAG, "IN Endpoint %d", epnum);
        uint8_t fifo_num = get_free_fifo();
        in_ep[epnum].diepctl &= ~(USB_D_TXFNUM1_M | USB_D_EPTYPE1_M | USB_DI_SETD0PID1 | USB_D_MPS1_M);
        in_ep[epnum].diepctl |= USB_D_USBACTEP1_M |
                                fifo_num << USB_D_TXFNUM1_S |
                                desc_edpt->bmAttributes.xfer << USB_D_EPTYPE1_S |
                                (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS ? (1 << USB_DI_SETD0PID1_S) : 0) |
                                xfer->max_size << 0;

        USB0.daintmsk |= (1 << (0 + epnum));

        uint16_t const allocated_size = (USB0.grxfsiz & 0x0000ffff) + 16;
        uint16_t const fifo_size = (EP_FIFO_SIZE/4 - allocated_size) / (EP_FIFO_NUM-1);
        uint32_t const fifo_offset = allocated_size + fifo_size*(fifo_num-1);

        USB0.dieptxf[epnum - 1] = (fifo_size << USB_NPTXFDEP_S) | fifo_offset;
    }
    return true;
}

void dcd_int_enable(uint8_t rhport)
{
    ESP_EARLY_LOGV(TAG, "Interrupt enable");
    esp_intr_alloc(ETS_USB_INTR_SOURCE, ESP_INTR_FLAG_LOWMED, (intr_handler_t) _dcd_int_handler, NULL, &usb_ih);
}

void dcd_int_disable(uint8_t rhport)
{
    ESP_EARLY_LOGV(TAG, "Interrupt disable");
    esp_intr_free(usb_ih);
}

static void _dcd_int_handler(void* arg)
{
    (void) arg;
    uint8_t const rhport = 0;

    const uint32_t int_msk = USB0.gintmsk;
    const uint32_t int_status = USB0.gintsts & int_msk;

    // Reset interrupt handling
    if (int_status & USB_USBRST_M) {
        ESP_EARLY_LOGV(TAG, "USB Reset detected");
        USB0.gintsts = USB_USBRST_M;
        bus_reset();
        dcd_event_bus_signal(rhport, DCD_EVENT_BUS_RESET, true);
    }

    // Enumeration done
    if (int_status & USB_ENUMDONE_M) {
        USB0.gintsts = USB_ENUMDONE_M;
        enum_done_processing();
        dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
    }

    // Setup packet received
    if (int_status & USB_RXFLVL_M) {
        // Mask out RXFLVL while reading data from FIFO
        USB0.gintmsk &= ~USB_RXFLVIMSK_M;
        
        uint32_t const status = USB0.grxstsp;
        uint8_t epnum = status & USB_CHNUM_M;
        uint8_t pktsts = (status & USB_PKTSTS_M) >> USB_PKTSTS_S;

        if (pktsts == 0x6) {
            // This indicates a SETUP packet
            ESP_EARLY_LOGV(TAG, "Setup packet received");
            
            // Copy setup packet from FIFO
            memcpy(_setup_packet, USB0.fifo[0], sizeof(_setup_packet));

            // Notify TinyUSB of the received setup packet
            dcd_event_setup_received(rhport, (uint8_t*)_setup_packet, true);
        }

        // Re-enable RXFLVL interrupt
        USB0.gintmsk |= USB_RXFLVIMSK_M;
    }

    // Handle other USB interrupts
    // (IN/OUT endpoints, SOF, etc.)
}


#endif // #if OPT_MCU_ESP32S2 || OPT_MCU_ESP32S3
