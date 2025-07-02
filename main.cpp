/*
// This code is used to convert the GNSS logger output csv file (.txt) 
// format to RINEX format (*.?o)
*          Copyright (C), All rights reserved to
*          Dr. Yang Gao's group.
// Authors:   Farzaneh Zangenehnejad and Yang Jiang from Dr. Yang Gao's group,
//            University of Calgary, Calgary, Canada 
// Contact Email: farzaneh.zangenehnej@ucalgary.ca and yang.jiang1@ucalgary.ca 
// Version : 2  (July 2025)
*/

#include <vector>
#include <stdio.h>
#include <time.h>
#include <string.h> 
#include <stdlib.h>
#include <math.h>
#include <cmath>

// Enter input and output file names and paths 
#define  INPUT_FILE   "D:\\px8.txt"  
#define  OUTPUT_FILE  "D:\\rnx_conv"

#define CLIGHT      299792458.0         /* Speed of light (m/s) */
#define LeapSecond      18              /* Leap seccond for 2021 */

#define MAXPRRUNCMPS  10               /* Maximum pseudorange rate (Doppler) uncertainty */
#define MAXTOWUNCNS   500              /* Maximum Tow uncertainty, 500 ns, for example */

#define MAX_LINE 1024

#define MAX_SYS 10
#define MAX_FRQ 5

#define SYS_GPS 1
#define SYS_GLO 3
#define SYS_GAL 6
#define SYS_BDS 5
#define SYS_QZS 4
#define SYS_IRN 7

#define RNX_VER "     3.04           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE"
#define RNX_PGM "UofC CSV2RINEX convertor                                    PGM / RUN BY / DATE "
#define RNX_APP "                                                            APPROX POSITION XYZ "
#define RNX_ANT "        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N"
#define RNX_END "                                                            END OF HEADER       "

// Refer to: https://android.googlesource.com/platform/hardware/libhardware/+/master/include/hardware/gps.h

#define GPS_MEASUREMENT_STATE_UNKNOWN       0
#define STATE_CODE_LOCK                     1//2^0  
#define STATE_TOW_DECODED                   8//2^3
#define STATE_TOW_KNOWN                 16384// maybe TOW not decoded but known from other sources. In contrast, if TOW is decoded then it is known

#define STATE_GLO_STRING_SYNC               64//2^6
#define STATE_GLO_TOD_KNOWN                 128//2^7

#define STATE_GAL_E1C_2ND_CODE_LOCK         2048//2^11
#define STATE_GAL_E1BC_CODE_LOCK            1024//2^10
#define STATE_GAL_E1B_PAGE_SYNC             4096//2^12  

#define GPS_ADR_STATE_UNKNOWN               0 
#define GPS_ADR_STATE_VALID                 1//2^0
#define GPS_ADR_STATE_RESET                 2//2^1
#define GPS_ADR_STATE_CYCLE_SLIP            4//2^2
#define GPS_ADR_STATE_HALF_CYCLE_RESOLVED   8//2^3
#define GPS_ADR_STATE_HALF_CYCLE_REPORTED  16//2^4

#define LLI_SLIP    0x01                /* LLI: cycle-slip */
#define LLI_HALFC   0x02                /* LLI: half-cycle not resovled */
#define LLI_BOCTRK  0x04                /* LLI: boc tracking of mboc signal */
#define LLI_HALFA   0x40                /* LLI: half-cycle added */
#define LLI_HALFS   0x80                /* LLI: half-cycle subtracted */

#define NEAR_ZERO	0.0001			        /* Threshold to judge if a float equals 0 */

struct gnss_sat
{
	long long ElapsedRealtimeMillis;
	long long time_nano;
	int leap_second;
	double time_uncertainty_nano;
	long long full_bias_nano;
	double bias_nano;
	double bias_uncertainty_nano;
	double drift_nano_per_second;
	double drift_uncertainty_nano_per_second;
	int hardware_clock_discountinuity_count;
	int svid;
	double time_offset_nano;
	int state;
	long long received_sv_time_nano;
	long long received_sv_time_uncertainty_nano;
	double cn0_dbhz;
	double pseudorange_rate_meter_per_second;
	double pseudorange_rate_uncertainty_meter_per_second;
	int accumulated_delta_range_state;
	double accumulated_delta_range_meter;
	double accumulated_delta_range_uncertainty_meter;
	double carrier_frequency_hz;
	long long carrier_cycle;
	double carrier_phase;
	double carrier_phase_uncertainty;
	int multipath_indicator;
	double snr_in_db;
	int constellation_type;
	double agc_db;
	double carrier_frequency_hz2;

	char signal_name[5];
	int sys;

	// Read the log file created by GnssLogger App in Android v.7 or higher 
	void parse_from(char* str)
	{
		char* token;
		int field_index = 0;
		token = strtok(str, ",");
		token = strtok(NULL, ","); 

		while (token != NULL) {
			switch (field_index) {
			case (0): ElapsedRealtimeMillis = atoll(token); break;
			case (1): time_nano = atoll(token); break;
			case (2): leap_second = atof(token); break;
			case (3): time_uncertainty_nano = atof(token); break;
			case (4): full_bias_nano = atoll(token); break;
			case (5): bias_nano = atof(token); break;
			case (6): bias_uncertainty_nano = atof(token); break;
			case (7): drift_nano_per_second = atof(token); break;
			case (8): drift_uncertainty_nano_per_second = atof(token); break;
			case (9): hardware_clock_discountinuity_count = atoi(token); break;
			case (10):svid = atoi(token); break;
			case (11):time_offset_nano = atof(token); break;
			case (12):state = atoi(token); break;
			case (13):received_sv_time_nano = atoll(token); break;
			case (14):received_sv_time_uncertainty_nano = atoll(token); break;
			case (15):cn0_dbhz = atof(token); break;
			case (16):pseudorange_rate_meter_per_second = atof(token); break;
			case (17):pseudorange_rate_uncertainty_meter_per_second = atof(token); break;
			case (18):accumulated_delta_range_state = atoi(token); break;
			case (19):accumulated_delta_range_meter = atof(token); break;
			case (20):accumulated_delta_range_uncertainty_meter = atof(token); break;
			case (21):carrier_frequency_hz = atof(token); break;
			case (22):carrier_cycle = atoll(token); break;
			case (23):carrier_phase = atof(token); break;
			case (24):carrier_phase_uncertainty = atof(token); break;
			case (25):multipath_indicator = atoi(token); break;
			case (26):snr_in_db = atof(token); break;
			case (27):constellation_type = atoi(token); break;
			default:break;
			}
			field_index++;
			token = strtok(NULL, ","); // Move to next token
		}
	}
};

struct gnss_epoch
{
	long long full_bias_nano;
	long long time_nano;
	double bia_nano;

	int nobs;
	gnss_sat* obs;
};

struct rnx_sat
{
	int sys;
	int prn;

	double p[MAX_FRQ];
	double l[MAX_FRQ];
	double d[MAX_FRQ];
	double s[MAX_FRQ];
	int lli[MAX_FRQ];
};

struct rnx_epoch
{
	double time[6];
	int sv;

	std::vector<rnx_sat*> sats;

	rnx_epoch()
	{
		memset(time, 0, sizeof(double) * 6);
		sv = 0;
	}
};

std::vector<gnss_epoch> epochs;
std::vector<rnx_epoch> rnx;
char signals[MAX_SYS][MAX_FRQ][5];
int nsignals[MAX_SYS] = { 0 };

char sys_code[5] = { 'G', 'R', 'E', 'C', 'J'};
int sys_code_function(int sys)
{
	int sys_n=-1;
	if (sys == 1) sys_n = 0;
	if (sys == 3) sys_n = 1;
	if (sys == 6) sys_n = 2;
	if (sys == 5) sys_n = 3;
	if (sys == 4) sys_n = 4;
	return sys_n;
}

int find_signal(int sys, char* sig)
{
	int sys_n = 0;
	sys_n = sys_code_function(sys);

	for (int i = 0; i < nsignals[sys_n]; i++)
	{
		if (strcmp(signals[sys_n][i], sig) == 0)
			return i;
	}
	return -1;
}

void add_signal(int sys, char* sig)
{
	if (find_signal(sys, sig) != -1) return;

	int sys_n = 0;
	sys_n = sys_code_function(sys);

	strcpy(signals[sys_n][nsignals[sys_n]], sig);
	nsignals[sys_n]++;
}

void print_rnx_epoch(FILE* fp, rnx_epoch e)
{
	
	fprintf(fp, "> %04d %02d %02d %02d %02d %10.7lf  0 %2d\n",
		(int)e.time[0], (int)e.time[1], (int)e.time[2], (int)e.time[3],
		(int)e.time[4], (e.time[5]*1e7l)/1e7l, e.sv);
	for (auto it = e.sats.begin(); it != e.sats.end(); it++)
	{
		
		int sys_n=0;
		sys_n = sys_code_function((*it)->sys);
 		fprintf(fp, "%c%02d", sys_code[sys_n] , (*it)->prn);
		int nsig = nsignals[sys_n];

		for (int i = 0; i < nsig; i++)
		{
			if ((*it)->p[i]) 
				fprintf(fp, "%14.3lf  ", (*it)->p[i]);
			else
				fprintf(fp, "                ");
			if ((*it)->l[i]) {
				char phase_lli[15]; // 14 chars + null terminator
				unsigned char lli = (*it)->lli[i] & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK);
				// Format value + LLI into string 
				snprintf(phase_lli, sizeof(phase_lli), "%13.3lf%1d", (*it)->l[i], lli);
				fprintf(fp, "%14s", phase_lli);
			}
			else {
				fprintf(fp, "              ");  // 14 spaces
			}
			if ((*it)->d[i])
				fprintf(fp, "%14.3lf  ", (*it)->d[i]);
			else
				fprintf(fp, "                ");
			if ((*it)->s[i])
				fprintf(fp, "%14.3lf  ", (*it)->s[i]);
			else
				fprintf(fp, "                ");
		}
		fprintf(fp, "\n");
	}
}

// Write the RINEX header section 
void print_rnx_header(FILE* fp)
{
	fprintf(fp, "%s\n", RNX_VER);
	fprintf(fp, "%s\n", RNX_PGM);
	fprintf(fp, "%s\n", RNX_APP);
	fprintf(fp, "%s\n", RNX_ANT);

	// Signal types
	char signal_line[MAX_LINE] = "";

	// Signals GPS, GLO, GAL, BDS, QZS
	for (int i = 0; i < 5; i++)
	{
		switch (nsignals[i])
		{
		case 0:
			break;
		case 1:
			sprintf(signal_line, "%c    %d C%-2s L%-2s D%-2s S%-2s                                      SYS / # / OBS TYPES ",
				sys_code[i],
				nsignals[i] * 4,
				signals[i][0] + 1, signals[i][0] + 1,
				signals[i][0] + 1, signals[i][0] + 1);
			fprintf(fp, "%s\n", signal_line);
			break;
		case 2:
			sprintf(signal_line, "%c    %d C%-2s L%-2s D%-2s S%-2s C%-2s L%-2s D%-2s S%-2s                      SYS / # / OBS TYPES ",
				sys_code[i],
				nsignals[i] * 4,
				signals[i][0] + 1, signals[i][0] + 1,
				signals[i][0] + 1, signals[i][0] + 1,
				signals[i][1] + 1, signals[i][1] + 1,
				signals[i][1] + 1, signals[i][1] + 1);
			fprintf(fp, "%s\n", signal_line);
			break;
		}
	}

	fprintf(fp, "%s\n", RNX_END);
}

// Function to compute GPS time from time_nano full_bias_nano and bias_nano
void  gpstime2ymdhms(long long *time_nano, long long *full_bias_nano, double *bias_nano,double *time) {

	// This the formula to compute the GPS time: GPS time = time Nano - (fullbiasnano + biasnano)[ns]
	long long   delta_time_nano = *time_nano - *full_bias_nano;  //in ns
	long long   delta_time_sec  = delta_time_nano / 1000000000LL; //full sec in second 
	long double delta_time_frac = ((long double)(delta_time_nano - delta_time_sec * 1000000000LL) - (long double)*bias_nano) / 1e9l; //fractional part 
	
	int HOURSEC = 3600, MINSEC = 60; /* Number of seconds in an hour and in a minute*/
	int DAYSEC = 86400;              /* Number of seconds in a day*/

	int monthDays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; // No of days in each month (not a leap year)
	int m = 1;
	int days = floor(delta_time_sec / DAYSEC) + 6; //days since 1980 / 1 / 1
	int	years = 1980.0;

	// Decrement days by a year at a time, until we have calculated the year :
	int leap = 1; // 1980 was a leap year
	while (days > leap + 365) {
		days = days - (leap + 365);
		years = years + 1;
		leap = ((years%4) == 0);// leap = 1 on a leap year, 0 otherwise
			// This works from 1901 till 2099, 2100 isn't a leap year (2000 is).
			// Calculate the year, ie time(1)
	}
	time[0] = years;

	// Decrement days by a month at a time, until we have calculated the month
	// Calculate the month, ie time(:, 2)
	int month = 1;
	if ((years % 4) == 0) { //% This works from 1901 till 2099
		monthDays[1] = 29;  // Make February have 29 days in a leap year
	}
	else {
		monthDays[1] = 28;
	}
	while (days > monthDays[month - 1]) {
		days = days - monthDays[month-1];
		month = month + 1;
	}
		
	time[1] = month;
	time[2] = days;
	
	int sinceMidnightSeconds = delta_time_sec%DAYSEC;
	time[3] = floor(sinceMidnightSeconds/HOURSEC);
	
	int lastHourSeconds = sinceMidnightSeconds%HOURSEC;
	time[4] = floor(lastHourSeconds/MINSEC);
	time[5] = (lastHourSeconds%MINSEC)+ delta_time_frac;
}

int main(int argc, char** argv[])
{
	// Parse the file into vector<gnss_epoch>
	FILE* fp = fopen(INPUT_FILE, "r");
	if (!fp) return 0;

	char line[MAX_LINE] = "";
	while (fgets(line, MAX_LINE, fp))
	{
		if ( strstr(line, "Raw,")!= 0 && strstr(line, "#") == 0) { //Read each line and substitute empty fields with 0
			gnss_epoch epoch;
			epoch.obs = (gnss_sat*)malloc(sizeof(gnss_sat) * 1);
			char* tok = NULL;
			char* newstr = NULL;
			char* oldstr = NULL;
			int   oldstr_len = 0;
			int   substr_len = 0;
			int   replacement_len = 0;
			substr_len = strlen(",,");
			replacement_len = strlen(",0,");

			newstr = line;
			while ((tok = strstr(newstr, ",,"))) {
				oldstr = newstr;
				oldstr_len = strlen(oldstr);
				newstr = (char*)malloc(sizeof(char) * (oldstr_len - substr_len + replacement_len + 1));

				if (newstr == NULL) {
					free(oldstr);
					return NULL;
				}

				memcpy(newstr, oldstr, tok - oldstr);
				memcpy(newstr + (tok - oldstr), ",0,", replacement_len);
				memcpy(newstr + (tok - oldstr) + replacement_len, tok + substr_len, oldstr_len - substr_len - (tok - oldstr));
				memset(newstr + oldstr_len - substr_len + replacement_len, 0, 1);
			}
			epoch.obs[0].parse_from(newstr);
			free(newstr);

			epochs.push_back(epoch);
		}
	}
	fclose(fp);

	// Find each constellation and signal type
	for (auto it = epochs.begin(); it != epochs.end(); it++) {
		gnss_sat* sat = it->obs + 0;
		switch (sat->constellation_type)
		{
		case 1: // GPS
			if (round(sat->carrier_frequency_hz / 1e4) == 157542)       /* GPS L1 1575420000 */ 
			{
				sat->sys = SYS_GPS;
				sprintf(sat->signal_name, "L1%c", 'C');
				add_signal(SYS_GPS, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e4) * 1e4;
			}
			else if (round(sat->carrier_frequency_hz / 1e4) == 117645)  /* GPS L5 1176450000 */
			{
				sat->sys = SYS_GPS;
				sprintf(sat->signal_name, "L5%c", 'Q');
				add_signal(SYS_GPS, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e4) * 1e4;
			}
			break;
		case 3: // GLO
			if (round(sat->carrier_frequency_hz / 1e7) == 160)         /* GLO L1 1602000000 */
			{
				sat->sys = SYS_GLO;
				sprintf(sat->signal_name, "L1%c", 'C');
				add_signal(SYS_GLO, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e2) * 1e2;
			}
			break;

		case 5: // BDS 
			if (round(sat->carrier_frequency_hz / 1e3) == 1561098)   /* BDS B1-2 1561097984 */
			{
				sat->sys = SYS_BDS;
				sprintf(sat->signal_name, "L2%c", 'I');
				add_signal(SYS_BDS, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e3) * 1e3;
			}
			if (round(sat->carrier_frequency_hz / 1e3) == 1176450)   /* BDS B2a 1176450000 */
			{
				sat->sys = SYS_BDS;
				sprintf(sat->signal_name, "L5%c", 'P');
				add_signal(SYS_BDS, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e3) * 1e3;
			}
			break;

		case 6: // GAL
			if (round(sat->carrier_frequency_hz / 1e4) == 157542)     /* GAL L1 1575420000 */
			{
				sat->sys = SYS_GAL;
				sprintf(sat->signal_name, "L1%c", 'C');
				add_signal(SYS_GAL, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e4) * 1e4;
			}
			else if (round(sat->carrier_frequency_hz / 1e4) == 117645) /* GAL E5a 1176450000 */
			{
				sat->sys = SYS_GAL;
				sprintf(sat->signal_name, "L5%c", 'X');
				add_signal(SYS_GAL, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e4) * 1e4;
			}
			break;

		case 4: // QZS
			sat->svid = sat->svid - 192;
			if (round(sat->carrier_frequency_hz / 1e4) == 157542)       /* QZSS L1 1575420000 */
			{
				sat->sys = SYS_QZS;
				sprintf(sat->signal_name, "L1%c", 'C');
				add_signal(SYS_QZS, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e4) * 1e4;
			}
			else if (round(sat->carrier_frequency_hz / 1e4) == 117645)  /* QZSS L5 1176450000 */
			{
				sat->sys = SYS_QZS;
				sprintf(sat->signal_name, "L5%c", 'Q');
				add_signal(SYS_QZS, sat->signal_name);
				sat->carrier_frequency_hz = round(sat->carrier_frequency_hz / 1e4) * 1e4;
			}
			break;
		}
	}

	// Compute full cycle time of measurement, in milliseonds
	unsigned __int64 allRxMillis_p = unsigned __int64((epochs.begin()->obs->time_nano - epochs.begin()->obs->full_bias_nano) * 1e-6);
	// allRxMillis is now accurate to one millisecond (because it's an integer)

	double check_clkdiscp = epochs.begin()->obs->hardware_clock_discountinuity_count;
	bool clkdiscp = true;
	
	int epo_bias = 0;
	int count = -1;
	rnx_epoch repoch;
	rnx_epoch repoch_pre; //previous epoch
	for (auto it = epochs.begin(); it != epochs.end(); it++)
	{
		count++;
		unsigned __int64 allRxMillis = unsigned __int64((it->obs->time_nano - it->obs->full_bias_nano) * 1e-6);
		
		// Anything within 1ms is considered same epoch :
		if (fabs(allRxMillis- allRxMillis_p) > NEAR_ZERO) {
			// Check and correct 4 ms delay/advance for Galileo (E1 and E5a)
			if (!rnx.empty()) {
				repoch_pre = rnx.back();
				if (repoch_pre.sv > 0&&repoch.sv > 0 && round(fabs(allRxMillis - allRxMillis_p)/1000)==1) {
					for (int j = 0; j < repoch.sv; j++) {
						int sys = repoch.sats[j]->sys;
						if (sys == SYS_GAL) {
							for (int k = 0; k < repoch_pre.sv; k++) {
								if (repoch.sats[j]->prn == repoch_pre.sats[k]->prn) {
									if (repoch.sats[j]->p[0] != 0 && repoch_pre.sats[k]->p[0] != 0 && (fabs(repoch.sats[j]->p[0] - repoch_pre.sats[k]->p[0] - 0.004 * CLIGHT) < 1500|| fabs(repoch.sats[j]->p[0] - repoch_pre.sats[k]->p[0] + 0.004 * CLIGHT) < 1500)) {
										int sign = (repoch.sats[j]->p[0]  - repoch_pre.sats[k]->p[0]) < 0 ? -1 : 1;
										repoch.sats[j]->p[0] = repoch.sats[j]->p[0] - sign * 0.004 * CLIGHT;
									}
									if (repoch.sats[j]->p[1] != 0 && repoch_pre.sats[k]->p[1] != 0 && (fabs(repoch.sats[j]->p[1] - repoch_pre.sats[k]->p[1] - 0.004 * CLIGHT) < 1500 || fabs(repoch.sats[j]->p[1] - repoch_pre.sats[k]->p[1] + 0.004 * CLIGHT) < 1500)) {
										int sign = (repoch.sats[j]->p[1]  - repoch_pre.sats[k]->p[1]) < 0 ? -1 : 1;
										repoch.sats[j]->p[1] = repoch.sats[j]->p[1] - sign * 0.004 * CLIGHT;
									}
								}
							}
						}
					}
				}
			}

			rnx.push_back(repoch);
			if (repoch.sv <= 4) {
				printf("Warning: Number of satellites is less than 4 in this epoch \n");
			}
			memset(repoch.time, 0, sizeof(double) * 6);	
			repoch.sv = 0;
			repoch.sats.clear();
			allRxMillis_p = allRxMillis;
			double check_clkdisc = it->obs->hardware_clock_discountinuity_count;
			if (fabs(check_clkdisc - check_clkdiscp) > NEAR_ZERO) {
				check_clkdiscp = check_clkdisc;
				clkdiscp = false;
				epo_bias = count;
			}
		}
			
		double time[6] = { 0,0,0,0,0,0 }; // Initialize array

		gpstime2ymdhms(&it->obs->time_nano, &epochs[epo_bias].obs->full_bias_nano, &epochs[epo_bias].obs->bias_nano, time);

		repoch.time[0] = time[0];
		repoch.time[1] = time[1];
		repoch.time[2] = time[2];
		repoch.time[3] = time[3];
		repoch.time[4] = time[4];
		repoch.time[5] = time[5]; 

		gnss_sat* obs = it->obs ;
		rnx_sat* sat = NULL;
		bool new_sat = false;
		bool available = false;
			
		if (obs->sys == SYS_GPS || obs->sys == SYS_BDS || obs->sys == SYS_QZS)
		{
			available = obs->state&STATE_CODE_LOCK && obs->state&STATE_TOW_DECODED;
			/*if (round(obs->carrier_frequency_hz / 1e4) == 117645) {
				available = obs->state & STATE_CODE_LOCK;
			}*/
		}
		else if (obs->sys == SYS_GLO)
		{
			available = obs->state&STATE_GLO_STRING_SYNC && obs->state&STATE_GLO_TOD_KNOWN;
		}
		else if (obs->sys == SYS_GAL)
		{
	     	available =((obs->state & STATE_GAL_E1C_2ND_CODE_LOCK) || ((obs->state & STATE_TOW_DECODED)));
			
			/*if (round(obs->carrier_frequency_hz / 1e4) == 117645) {
				available = obs->state & STATE_TOW_DECODED;
			}*/
		}

		if (!available) continue; /* Reject bad observations with invalid state */

		if (obs->pseudorange_rate_uncertainty_meter_per_second> MAXPRRUNCMPS || obs->received_sv_time_uncertainty_nano>MAXTOWUNCNS) {
			continue; /* Reject bad observations */
		}

		for (int j = 0; j < repoch.sv; j++)
		{
			if (repoch.sats[j]->sys == obs->sys &&
				repoch.sats[j]->prn == obs->svid)
			{
				sat = repoch.sats[j];
				break;
			}
		}

		if (!sat)
		{
			sat = new rnx_sat();
			new_sat = true;

			sat->sys = obs->sys;
			sat->prn = obs->svid;
		}

		
		int frq = find_signal(obs->sys, obs->signal_name);
		if (frq == -1) continue;

		double wavl = CLIGHT / obs->carrier_frequency_hz; /* Compute the wavelength as Lambda= c/f */
		double wavl_inv = 1.0 / wavl;
			
		long long time_from_gps_start = long long(obs->time_nano)-long long(epochs[epo_bias].obs->full_bias_nano) + long long(obs->time_offset_nano) ;

		long double receive_second = 0.0l;  /* Initialize time of reception */
		
		long long send_second = ((long long)obs->received_sv_time_nano); /* Time of transmission in ns */

		long double DayNonano = 0.0l;
		unsigned __int64  WeekNonano = 0.0l;
		long double milliSecondNumberNanos = 0.0l;
		// https://www.gsa.europa.eu/system/files/reports/gnss_raw_measurement_web_0.pdf pp.21-22

		switch (obs->sys)
		{
		case SYS_GPS:
			WeekNonano = long long(floor(-(long double)obs->full_bias_nano * 1e-9l / 604800.0l)) ;
			receive_second =  long long(time_from_gps_start) -long long(WeekNonano*604800* 1e9l) ; /* Time of reception in ns */

			break;
		case SYS_GLO:
			DayNonano = long long(floor(long long (-obs->full_bias_nano)  / long long( 86400.00 * 1e9l))) * long long(86400.00 * 1e9l);
			receive_second = long long(time_from_gps_start) - long long(DayNonano)  + long long ((3*3600  - LeapSecond) * 1e9l); /* Time of reception in ns */

			break;
		case SYS_BDS:	
			WeekNonano = long long(floor(-(long double)obs->full_bias_nano * 1e-9l / 604800.0l));
			receive_second = long long(time_from_gps_start) - long long(WeekNonano * 604800 * 1e9l)-long long( 14*1e9l)  ;  /* Time of reception in ns */

			break;
		case SYS_GAL:
			WeekNonano = long long(floor(-(long double)obs->full_bias_nano * 1e-9l / 604800.0l));
			receive_second = long long(time_from_gps_start) - long long(WeekNonano * 604800 * 1e9l); /* Time of reception in ns */

			break;
		case SYS_QZS:
			WeekNonano = long long(floor(-(long double)obs->full_bias_nano * 1e-9l / 604800.0l));
			receive_second = long long(time_from_gps_start) - long long(WeekNonano * 604800 * 1e9l); /* Time of reception in ns */

			break;
		}

		/* pr_second is the time difference between time of reception and time of transmission in seconde. */
		long double pr_second = long double(long long (receive_second) - long long(send_second))* 1e-9l - long double(epochs[epo_bias].obs->bias_nano * 1e-9l);
		
		/* Check for week rollover in receive_second (time of reception) */
		if (pr_second > 604800 / 2) {
			double delS = round(pr_second / 604800) * 604800;
			pr_second = pr_second - delS;
			/* pr_second are in the range[-604800/2:604800/2];
			 Check that common bias is not huge(like, bigger than 10s) */
			int maxBiasSec = 10;
			if (pr_second > maxBiasSec) printf("Failed to correct week rollover\n");
			else printf("Week rollover detected and corrected \n");
		}
			
		if ((obs->sys==SYS_GPS||obs->sys==SYS_GAL||obs->sys==SYS_BDS||obs->sys == SYS_QZS) && pr_second>604800) {
			pr_second = fmodl(pr_second, 604800.0l);
		}
		if (obs->sys == SYS_GLO && pr_second > 86400) {
			pr_second = fmodl(pr_second, 86400.0l);
		}
		if (pr_second > 0.5|| pr_second <0)	continue; 
		if (obs->sys == SYS_GLO && obs->svid > 80) { continue;} // Delete some odd GLONASS numbers larger than 80 
		
		sat->p[frq] = (long double)pr_second * CLIGHT;                    // Pseudorange measurement
		sat->d[frq] = -obs->pseudorange_rate_meter_per_second * wavl_inv; // Carrier-phase measurement
		sat->l[frq] = (obs->accumulated_delta_range_meter * wavl_inv);    // Doppler measurement
		sat->s[frq] = obs->cn0_dbhz;                                      // C/N0 measurement
			
		if (obs->accumulated_delta_range_state& GPS_ADR_STATE_UNKNOWN) {
			sat->l[frq] = 0;
		}

		if ((obs->accumulated_delta_range_state & GPS_ADR_STATE_HALF_CYCLE_REPORTED) && !(obs->accumulated_delta_range_state & GPS_ADR_STATE_HALF_CYCLE_RESOLVED)) {
			sat->lli[frq] = LLI_HALFC;
		}
		if (obs->accumulated_delta_range_state & GPS_ADR_STATE_CYCLE_SLIP) {
			sat->lli[frq] = LLI_SLIP;
		}

		if (new_sat)
		{
			repoch.sats.push_back(sat);
			repoch.sv++;
		}		
	}
	rnx.push_back(repoch);

	// Build the RINEX file (output)
	char rinex_name[512] = "";
	strcpy(rinex_name, OUTPUT_FILE);
	
	for (int i = 0; i < (int)strlen(rinex_name); i++) {
		if (rinex_name[i] == '.') {
			rinex_name[i] = '\0';
			break;
		}
	}
	char ext[10] = "";
	sprintf(ext, ".%02do", (int)rnx[0].time[0] - 2000);

	strcat(rinex_name, ext);

	FILE* fpw = fopen(rinex_name, "w");

	print_rnx_header(fpw);

	for (auto it = rnx.begin(); it != rnx.end(); it++)
	{
		if (it->sv > 0)
		{
			print_rnx_epoch(fpw, *it);
		}
	}

	fclose(fpw);
}



