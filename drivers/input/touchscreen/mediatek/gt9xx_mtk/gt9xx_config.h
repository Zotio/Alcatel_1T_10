#ifndef _GT9XX_CONFIG_H_
#define _GT9XX_CONFIG_H_

/* ***************************PART2:TODO define********************************** */
/* STEP_1(REQUIRED):Change config table. */
/*TODO: puts the config info corresponded to your TP here, the following is just
a sample config, send this config should cause the chip cannot work normally*/
#define CTP_CFG_GROUP0 {\
0x4A,0x20,0x03,0x00,0x05,0x0A,0x05,0x00,\
0x01,0x19,0x3C,0x0F,0x48,0x28,0x03,0x05,\
0x00,0x00,0xE8,0x02,0x00,0x00,0x08,0x16,\
0x15,0x18,0x14,0x90,0x2F,0x9A,0x14,0x16,\
0x15,0x0E,0x00,0x00,0x00,0xDB,0x33,0x1C,\
0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x42,0x0C,0x2C,0x94,0xC5,0x02,\
0x07,0x00,0x00,0x04,0xAD,0x0D,0x00,0x8E,\
0x11,0x00,0x75,0x17,0x00,0x65,0x1E,0x00,\
0x5B,0x26,0x00,0x5B,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x23,0x0F,0x2D,0x28,0x0A,0xA4,\
0x00,0x03,0x01,0x0B,0x00,0x00,0x00,0x00,\
0x06,0x07,0x08,0x09,0x0C,0x0D,0x0E,0x0F,\
0x10,0x11,0x14,0x15,0x16,0x17,0x18,0x19,\
0x1A,0x1B,0x1C,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,\
0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0E,\
0x0F,0x10,0x11,0x12,0x13,0x14,0x19,0x1B,\
0x1C,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,\
0x25,0x26,0x27,0x28,0x29,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x1D,0x01\
}

#define GTP_CFG_GROUP0_CHARGER {\
}

/* TODO puts your group2 config info here,if need. */
#define CTP_CFG_GROUP1 {\
}

/* TODO puts your group2 config info here,if need. */
#define GTP_CFG_GROUP1_CHARGER {\
}

/* TODO puts your group3 config info here,if need. */
#define CTP_CFG_GROUP2 {\
0x48,0x20,0x03,0x00,0x05,0x0A,0x05,0x08,\
0x01,0x19,0x28,0x0F,0x34,0x1E,0x03,0x05,\
0x00,0x00,0x00,0x02,0x00,0x00,0x08,0x18,\
0x1A,0x1E,0x14,0x90,0x2F,0x9A,0x14,0x16,\
0x15,0x0E,0x00,0x00,0x00,0xDB,0x43,0x1C,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x42,0x0C,0x3C,0x94,0xC5,0x02,\
0x07,0x00,0x00,0x04,0x9E,0x0E,0x00,0x83,\
0x13,0x00,0x6D,0x1B,0x00,0x5F,0x25,0x00,\
0x54,0x33,0x00,0x54,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x23,0x08,0x45,0x44,0x0A,0xA4,\
0x00,0x00,0x01,0x0B,0x00,0x00,0x00,0x00,\
0x06,0x07,0x08,0x09,0x0C,0x0D,0x0E,0x0F,\
0x10,0x11,0x14,0x15,0x16,0x17,0x18,0x19,\
0x1A,0x1B,0x1C,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,\
0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0E,\
0x0F,0x10,0x11,0x12,0x13,0x14,0x29,0x28,\
0x27,0x26,0x25,0x24,0x23,0x22,0x21,0x20,\
0x1F,0x1E,0x1C,0x1B,0x19,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0xFD,0x01\
}

/* TODO puts your group3 config info here,if need. */
#define GTP_CFG_GROUP2_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 3 here, if needed */
#define CTP_CFG_GROUP3 {\
}

/* TODO puts your group3 config info here,if need. */
#define GTP_CFG_GROUP3_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 4 here, if needed */
#define CTP_CFG_GROUP4 {\
}

/* TODO puts your group4 config info here,if need. */
#define GTP_CFG_GROUP4_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 5 here, if needed */
#define CTP_CFG_GROUP5 {\
}

/* TODO puts your group5 config info here,if need. */
#define GTP_CFG_GROUP5_CHARGER {\
}


#endif /* _GT1X_CONFIG_H_ */