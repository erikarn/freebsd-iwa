/*-
 * Copyright (c) 2014 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Driver for Intel WiFi Link 7260 series 802.11n/802.11ac adapters.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_wlan.h"
//#include "opt_iwa.h"

#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/limits.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/clock.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_ratectl.h>

#include <dev/iwa/if_iwa_debug.h>

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwavar.h>

/*
 * XXX TODO: pull out the firmware bits completely from the
 * softc so this file doesn't require if_athvar.h to load in.
 */


/*
 * Load in the firmware for the NIC.
 *
 * Returns 0 if the firmware was grabbed, non-zero if there
 * was a problem.
 */
int
if_iwa_firmware_load(struct iwa_softc *sc, struct iwa_fw_info *fw,
    const char *fw_name)
{
        const struct firmware *fwh;
        int error;

        /* Open */
        fwh = firmware_get(fw_name);
        if (fwh == NULL) {
                device_printf(sc->sc_dev, "%s: failed to read firmware\n", __func__);
                return (EINVAL);
        }

        fw->fw_rawsize = fwh->datasize;
        /*
         * Well, this is how the Linux driver checks it ....
         */
        if (fw->fw_rawsize < sizeof(uint32_t)) {
                device_printf(sc->sc_dev,
                    "firmware too short: %zd bytes\n", fw->fw_rawsize);
                error = EINVAL;
                goto out;
        }

        /* some sanity */
        if (fw->fw_rawsize > IWM_FWMAXSIZE) {
                device_printf(sc->sc_dev,
                    "firmware size is ridiculous: %zd bytes\n",
                    fw->fw_rawsize);
                error = EINVAL;
                goto out;
        }

        /* Read the firmware. */
        fw->fw_rawdata = malloc(fw->fw_rawsize, M_TEMP, M_WAITOK);
        if (fw->fw_rawdata == NULL) {
                device_printf(sc->sc_dev,
                    "not enough memory to stock firmware %s\n", fw_name);
                error = ENOMEM;
                goto out;
        }

        memcpy(fw->fw_rawdata, fwh->data, fw->fw_rawsize);
        error = 0;

 out:
        /* caller will release memory, if necessary */

        firmware_put(fwh, FIRMWARE_UNLOAD);
        return error;
}
