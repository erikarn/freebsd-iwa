#ifndef	__IF_IWAREG_H__
#define	__IF_IWAREG_H__

/*
 * Register access macros.
 */
#define	IWA_LSB(x)	((((x) - 1) & (x)) ^ (x))

#define	IWA_REG_READ(sc, reg)						\
	bus_space_read_4((sc)->sc_st, (sc)->sc_sh, (reg))

#define	IWA_REG_READ_1(sc, reg)						\
	bus_space_read_1((sc)->sc_st, (sc)->sc_sh, (reg))

#define	IWA_REG_WRITE(sc, reg, val)					\
	bus_space_write_4((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define	IWA_REG_WRITE_1(sc, reg, val)					\
	bus_space_write_1((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define	IWA_REG_SETBITS(sc, reg, mask)					\
        IWA_WRITE(sc, reg, IWA_READ(sc, reg) | (mask))

#define	IWA_REG_CLRBITS(sc, reg, mask)					\
        IWA_WRITE(sc, reg, IWA_READ(sc, reg) & ~(mask))

#define	IWA_REG_BARRIER_WRITE(sc)					\
	bus_space_barrier((sc)->sc_st, (sc)->sc_sh, 0, (sc)->sc_sz,	\
            BUS_SPACE_BARRIER_WRITE)

#define	IWA_REG_BARRIER_READ_WRITE(sc)					\
	bus_space_barrier((sc)->sc_st, (sc)->sc_sh, 0, (sc)->sc_sz,	\
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE)

#endif	/* __IF_IWAREG_H__ */
