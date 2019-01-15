#define CAPACITY_MAX			1000
#define CAPACITY_MAX_MARGIN     30
#define CAPACITY_MIN			0

static sec_bat_adc_table_data_t temp_table[] = {
        {25950, 900},
        {26108, 850},
        {26515, 800},
        {26810, 750},
        {27203, 700},
        {27590, 650},
        {28085, 600},
        {28679, 550},
        {29343, 500},
        {30099, 450},
        {30962, 400},
        {31874, 350},
        {32865, 300},
        {33886, 250},
        {34996, 200},
        {36145, 150},
        {37148, 100},
        {38164, 50},
        {38432, 20},
        {38890, 0},
        {39156, -20},
        {39420, -40},
        {39650, -50},
        {40606, -100},
        {41393, -150},
        {41890, -200},
};

#define TEMP_HIGH_THRESHOLD_EVENT  580
#define TEMP_HIGH_RECOVERY_EVENT   530
#define TEMP_LOW_THRESHOLD_EVENT   (-50)
#define TEMP_LOW_RECOVERY_EVENT    0
#define TEMP_HIGH_THRESHOLD_NORMAL 580
#define TEMP_HIGH_RECOVERY_NORMAL  530
#define TEMP_LOW_THRESHOLD_NORMAL  (-50)
#define TEMP_LOW_RECOVERY_NORMAL   0
#define TEMP_HIGH_THRESHOLD_LPM    580
#define TEMP_HIGH_RECOVERY_LPM     530
#define TEMP_LOW_THRESHOLD_LPM     (-50)
#define TEMP_LOW_RECOVERY_LPM      0
