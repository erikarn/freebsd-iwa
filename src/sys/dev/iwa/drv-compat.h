#ifndef	__IWA_DRV_COMPAT_H__
#define	__IWA_DRV_COMPAT_H__

/*
 * These are compatibility definitions for the driver source in
 * iwl/ .
 *
 * It's not a linux compatibility layer - we're not going for
 * a straight "pretend it's all linux" port.
 * But some basic typedefs and config #define's make life easier.
 */

#define	BIT(x)		(1 << (x))
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint16_t u16;
typedef uint8_t u8;

typedef int8_t s8;
typedef int8_t __s8;

/* XXX le == signed or unsigned? */
typedef uint8_t __le8;
typedef uint16_t __le16;
typedef uint32_t __le32;
typedef uint64_t __le64;

typedef uint32_t __be32;

#define	le32_to_cpu(x)	le32toh(x)
#define	le32_to_cpup(x)	le32toh(*(x))

#define	le16_to_cpup(x)	le16toh(*(x))

#define	ETH_ALEN	6

#define	dma_addr_t bus_addr_t

/*
 * XXX XXXGPL this is from linux mac80211.h - whatever uses this needs
 * to be eventually turned into the freebsd SMPS stuff - and
 * freebsd's SMPS stuff needs to be an actual enum.
 */
/**
 * enum ieee80211_smps_mode - spatial multiplexing power save mode
 *
 * @IEEE80211_SMPS_AUTOMATIC: automatic
 * @IEEE80211_SMPS_OFF: off
 * @IEEE80211_SMPS_STATIC: static
 * @IEEE80211_SMPS_DYNAMIC: dynamic
 * @IEEE80211_SMPS_NUM_MODES: internal, don't use
 */
enum ieee80211_smps_mode {
        IEEE80211_SMPS_AUTOMATIC,
        IEEE80211_SMPS_OFF,
        IEEE80211_SMPS_STATIC,
        IEEE80211_SMPS_DYNAMIC,

        /* keep last */
        IEEE80211_SMPS_NUM_MODES,
};

#define	IEEE80211_MAX_SSID_LEN 32

/*
 * XXX XXXGPL from Linux include/linux/stringify.h
 */

/* Indirect stringification.  Doing two levels allows the parameter to be a
 * macro itself.  For example, compile with -DFOO=bar, __stringify(FOO)
 * converts to "bar".
 */

#define __stringify_1(x...)     #x
#define __stringify(x...)       __stringify_1(x)

#define	IEEE80211_BAND_2GHZ	0
#define	IEEE80211_BAND_5GHZ	1

/* XXX TODO: should turn this into something useful */
#define	WARN_ON_ONCE(x) do { if ( (!x) ) { printf("%s:%d: %s\n", __FILE__, __LINE__, #x); }} while (0)

#endif	/* __IWA_DRV_COMPAT_H__ */
