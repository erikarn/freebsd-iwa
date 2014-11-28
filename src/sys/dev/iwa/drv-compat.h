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


/*
 * XXX this is from linux mac80211.h - whatever uses this needs
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


#endif	/* __IWA_DRV_COMPAT_H__ */
