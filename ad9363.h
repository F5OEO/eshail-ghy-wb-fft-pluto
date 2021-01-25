/*
 *  AD9363 IIO Wrapper for PlutoSDR
 *
 * Copyright (C) 2020 Evariste F5OEO
 * Based on IIO Samples
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
 #include <iio.h>
#include "iio-private.h"
#include "rrcfir.h"
#include <time.h>
#include <unistd.h>

#define MIN_ADC_CLK (25e6/12.0)
#define MAX_ADC_CLK 61.44e6

 /* static scratch mem for strings */
static char tmpstr[64];
 struct iio_context *m_ctx = NULL;
 struct iio_device *m_dev = NULL;
 //Channels variable
static struct iio_buffer *m_rxbuf = NULL;
static struct iio_buffer *m_txbuf = NULL;

struct iio_device *m_tx = NULL;
static struct iio_channel *m_tx0_i = NULL;
static struct iio_channel *m_tx0_q = NULL;

struct iio_device *m_rx = NULL;
static struct iio_channel *m_rx0_i = NULL;
static struct iio_channel *m_rx0_q = NULL;


static int m_max_len; // Maximum size of the buffer
static int m_offset;  // Current offset into the buffer


/* RX is input, TX is output */
enum iodev
{
	RX,
	TX
};

#ifndef TYPE_CPLX
typedef struct {
	short re;
	short im;
}sfcmplx;
#endif

#define FMC_ERROR(expr)                                                                 \
	{                                                                                   \
		if (!(expr))                                                                    \
		{                                                                               \
			(void)fprintf(stderr, "FMC_ERRORion failed (%s:%d)\n", __FILE__, __LINE__); \
			(void)abort();                                                              \
		}                                                                               \
	}


/* cleanup and exit */
void fmc_shutdown()
{

	if (m_ctx != NULL)
	{
		
		//printf("* Destroying buffers\n");
		if (m_rxbuf)
		{
			iio_buffer_destroy(m_rxbuf);
			m_rxbuf = NULL;
		}
		if (m_txbuf)
		{
			iio_buffer_destroy(m_txbuf);
			m_txbuf = NULL;
		}

		//printf("* Disabling streaming channels\n");
		if (m_rx0_i)
		{
			iio_channel_disable(m_rx0_i);
		}
		if (m_rx0_q)
		{
			iio_channel_disable(m_rx0_q);
		}
		if (m_tx0_i)
		{
			iio_channel_disable(m_tx0_i);
		}
		if (m_tx0_q)
		{
			iio_channel_disable(m_tx0_q);
		}

		//printf("* Destroying context\n");
		if (m_ctx)
		{
			iio_context_destroy(m_ctx);
		}
	}
}

/* check return value of attr_write function */
static void errchk(int v, const char *what)
{
	if (v < 0)
	{
		fprintf(stderr, "FMC_ERROR %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
		fmc_shutdown();
	}
}
/* read attribute: long long int */
static void rd_ch_lli(struct iio_channel *chn, const char *what, long long *val)
{
	errchk(iio_channel_attr_read_longlong(chn, what, val), what);
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char *what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char *what, const char *str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char *get_ch_name(const char *type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device *get_ad9361_phy(struct iio_context *ctx)
{
	struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
	FMC_ERROR(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
	switch (d)
	{
	case TX:
		*dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
		return *dev != NULL;
	case RX:
		*dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
		return *dev != NULL;
	default:
		FMC_ERROR(0);
		return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d)
	{
	case RX:
		*chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false);
		return *chn != NULL;
	case TX:
		*chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);
		return *chn != NULL;
	default:
		FMC_ERROR(0);
		return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
	switch (d)
	{
		// LO chan is always output, i.e. true
	case RX:
		*chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true);
		return *chn != NULL;
	case TX:
		*chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true);
		return *chn != NULL;
	default:
		FMC_ERROR(0);
		return false;
	}
}

bool fmc_init_name(char *PlutoName)
{
	m_ctx = iio_create_default_context();
	if (m_ctx == NULL)
		m_ctx = iio_create_local_context();
	if (m_ctx == NULL)
		m_ctx = iio_create_network_context(PlutoName);
    if (m_ctx == NULL)
		m_ctx = iio_create_network_context(PlutoName);
	if (m_ctx == NULL)
	{
		fprintf(stderr, "No Pluto Found. Exiting\n");
		exit(1);
	}

	if (iio_context_get_devices_count(m_ctx) <= 0)
	{
		FMC_ERROR("No Pluto Found. Exiting\n")
		exit(1);
	}

    m_dev= get_ad9361_phy(m_ctx);
	iio_device_debug_attr_write_bool(m_dev, "adi,frequency-division-duplex-mode-enable", 1); // We still want FDD
	iio_device_debug_attr_write_bool(m_dev, "adi,gpo-manual-mode-enable", 1);
	iio_device_debug_attr_write(m_dev, "direct_reg_access", "0x26 0x10"); // Manual GPIO SET

	iio_device_attr_write(m_dev, "calib_mode", "manual_tx_quad");

	FMC_ERROR(get_ad9361_stream_dev(m_ctx, TX, &m_tx) && "No tx dev found");
	FMC_ERROR(get_ad9361_stream_dev(m_ctx, RX, &m_rx) && "No rx dev found");
	// Carefull this is a workaround : Need to set calibmod before streaming_channel, else calibration is done

}

bool fmc_init()
{ 
	char Pluto[255]="pluto.local";
	fmc_init_name(Pluto);
}

/*
int ad9363_set_trx_fir_enable(struct iio_device *dev, int enable)
{
    
    int ret = iio_device_attr_write_bool(dev,"in_out_voltage_filter_fir_en", !!enable);
    if(ret<0) FMC_ERROR("out fir. Exiting\n");
        
    
	if (ret < 0)
		ret = iio_channel_attr_write_bool(iio_device_find_channel(dev, "out", false), "voltage_filter_fir_en", !!enable);

	return 0;
}
*/
int ad9363_set_fir_enable(struct iio_device *dev, int enable,bool tx)
{
    int ret=0;
	if(tx)
	{
    	 ret = iio_device_attr_write_bool(dev,"out_voltage_filter_fir_en", !!enable);
	}	 
	else
	{
		ret = iio_device_attr_write_bool(dev,"in_voltage_filter_fir_en", !!enable);
	}
	    
	return 0;
}

int ad9363_set_trx_fir_enable( int enable)
{
	ad9363_set_fir_enable(m_dev,enable,true);
	ad9363_set_fir_enable(m_dev,enable,false);
	return true;
}

int ad9361_get_trx_fir_enable(struct iio_device *dev, int *enable)
{
	bool value;

	int ret = iio_device_attr_read_bool(dev, "in_out_voltage_filter_fir_en", &value);

	if (ret < 0)
		ret = iio_channel_attr_read_bool(iio_device_find_channel(dev, "out", false),
										 "voltage_filter_fir_en", &value);

	if (!ret)
		*enable = value;

	return ret;
}

void fmc_set_frequency(double freq,bool Tx)
{
	struct iio_channel *chn = NULL;
	if (m_ctx == NULL)
		return;
	if (!get_lo_chan(m_ctx, Tx?TX:RX, &chn))
	{
		return;
	}
	wr_ch_lli(chn, "frequency", (long long int)freq);
}

#define LOOPBACK_DISABLE 0
#define LOOPBACK_TX2RX 1
#define LOOPBACK_RX2TX 2
void fmc_set_loopback(bool enable,int Type)
{
	//Type=LOOPBACK_RX2TX
    if(enable)
	    iio_device_debug_attr_write_longlong(m_dev,"loopback",Type);
    else
    {
        iio_device_debug_attr_write_longlong(m_dev,"loopback",LOOPBACK_DISABLE);
    }
}

void fmc_set_prbs(bool enable)
{
    if(enable)
	    iio_device_debug_attr_write_longlong(m_dev,"bist_prbs",1); //Injection prbs on Tx
    else
    {
        iio_device_debug_attr_write_longlong(m_dev,"bist_prbs",0);
    }
}

void fmc_set_dds_tone(bool enable)
{
    if(enable)
	    iio_device_debug_attr_write(m_dev,"bist_tone","1 3 0 0"); //Injection prbs on Tx
    else
    {
		iio_device_debug_attr_write(m_dev,"bist_tone","0 0 0 0"); //Injection prbs on Tx
        //iio_device_debug_attr_write_longlong(m_dev,"bist_prbs",0);
    }
}

void fmc_set_tx_level(double level) //
{
    if(level>0) level=0;
    if(level<-89.75) level=-89.75;
	if (m_ctx == NULL)
		return;
	
	struct iio_channel *chn = NULL;
	if (!get_phy_chan(m_ctx, TX, 0, &chn))
	{
		return;
	}
	wr_ch_lli(chn, "hardwaregain", (long long int)level);
}

void fmc_set_rx_agcmode(bool agc) 
{
	struct iio_channel *chn = NULL;
	if (!get_phy_chan(m_ctx, RX, 0, &chn)) // Fixme ! There are 2 gain stages, here we get only zero stage
	{
		return;
	}
    
   if(agc)
		wr_ch_str(chn, "gain_control_mode", "fast_attack");	
	else	
    	wr_ch_str(chn, "gain_control_mode", "manual");
}


double fmc_get_rx_gain()
{
	struct iio_channel *chn = NULL;
	if (!get_phy_chan(m_ctx, RX, 0, &chn)) // Fixme ! There are 2 gain stages, here we get only zero stage
	{
		return 0;
	}
	double gain=0;
	iio_channel_attr_read_double(chn,"hardwaregain",&gain);
	return gain;

} 

void fmc_set_rx_level(double level) //
{
    if(level>73) level=73;
    if(level<-10) level=-10;
	if (m_ctx == NULL)
		return;
	
	struct iio_channel *chn = NULL;
	if (!get_phy_chan(m_ctx, RX, 0, &chn)) // Fixme ! There are 2 gain stages, here we get only zero stage
	{
		return;
	}
   // wr_ch_str(chn, "gain_control_mode", "slow_attack");
	wr_ch_str(chn, "gain_control_mode", "manual");
    wr_ch_lli(chn, "hardwaregain", (long long int)level);

}

void fmc_set_analog_lpf(double bw,bool tx)
{

	struct iio_channel *chn = NULL;
	if (!get_phy_chan(m_ctx, tx?TX:RX, 0, &chn))
		return;
	wr_ch_lli(chn, "rf_bandwidth", (int64_t)bw);
}

void fmc_load_tx_filter(double *firrx,double *firtx, int taps, int ratio, bool enable,float max,bool db6boost)
{
	//db6boost=false;
	int ModeRxTx=0;
	
	if (m_ctx == NULL)
		return;

	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);
	ad9363_set_trx_fir_enable(false);
    

	if (!enable)
		return; //No FIR, NEITHER upsample*/

	int buffsize = 8192;
	char *buf = (char *)malloc(buffsize);
	int clen = 0;
	clen += snprintf(buf + clen, buffsize - clen, "RX 3 GAIN 0 DEC %d\n", ratio); //The filter provides a fixed +6dB gain to maximize dynamic range, so the programmable gain is typically set to -6dB to produce a net gain of 0dB
	if(db6boost)
		clen += snprintf(buf + clen, buffsize - clen, "TX 3 GAIN 0 INT %d\n", ratio); //-6db seems better
	else
		clen += snprintf(buf + clen, buffsize - clen, "TX 3 GAIN -6 INT %d\n", ratio); //-6db seems better
		
	short coefrx=0;
	for (int i = 0; i < taps; i++)
	{
		if(i==taps/2) coefrx=0x7FFF; else coefrx=0;
		//clen += snprintf(buf + clen, buffsize - clen, "%d,%d\n", (short)(0x7FFF * firtx[i]/max),(short)(0x7FFF * firrx[i]/max)); //Fixme ! 1FFF instead of 0x3FFF seems better but should have to be inspect
		clen += snprintf(buf + clen, buffsize - clen, "%d,%d\n", (short)(0x7FFF * firtx[i]/max),coefrx); //Fixme ! 1FFF instead of 0x3FFF seems better but should have to be inspect
	}	
	clen += snprintf(buf + clen, buffsize - clen, "\n");

	//FMC_ERROR(iio_device_attr_write_bool(dev, "out_voltage_filter_fir_en", false));

	FMC_ERROR(iio_device_attr_write_raw(dev, "filter_fir_config", buf, clen));
	//ad9363_set_fir_enable(dev,enable,true); //FixMe only Tx right now ! Shoudl be related to whatwe want to do
	ad9363_set_trx_fir_enable(dev); //Enable Tx & RX FIR
	//ad9363_set_fir_enable(dev,false,false); //disable Rx FIR
	//FMC_ERROR(iio_device_attr_write_bool(dev, "out_voltage_filter_fir_en", true));

	free(buf);
}

double nullfir[128];
enum {typerrc,typelpf,unity};
void fmc_load_rrc_filter(int ratio, int hardupsample,float rolloff,int type,float digitalgain)
{

	
    double firrx[128];
    double firtx[128];
	double max=1;
    fprintf(stderr,"Build filter with %d ratio (up/down)sample\n",ratio);

	int NbTaps;
	if(hardupsample>1)
	{
		NbTaps = 127;
	}	
	else
	{
		NbTaps = 63;
	}
		
    switch(type)
    {
        case typerrc:
		{
			 
			
            build_rrc_filter(firtx,rolloff,NbTaps,ratio);
            build_rrc_filter(firrx,rolloff,NbTaps,ratio);
			max=set_filter_gain(firtx,1,NbTaps);
			firrx[NbTaps]=0.0;
    		firtx[NbTaps]=0.0;
			fprintf(stderr, "Max FIR =%f\n", max);

			// For FIR !!!!
			max=max*digitalgain;
			
			fmc_load_tx_filter(firrx,firtx, NbTaps+1, hardupsample, true,max,false);
		}	
            break;
        case typelpf:
            fprintf(stderr,"LPF rolloff %f\n",rolloff);
            build_lpf_filter(firtx,(rolloff)/(float)(ratio),NbTaps);
            build_lpf_filter(firrx,(rolloff)/(float)(ratio),NbTaps);
			max=set_filter_gain(firtx,ratio,NbTaps);
			set_filter_gain(firrx,1,NbTaps);
			max=max*digitalgain;
			firrx[NbTaps]=0.0;
    		firtx[NbTaps]=0.0;
			fprintf(stderr, "Max FIR =%f\n", max);
			fmc_load_tx_filter(firrx,firtx, NbTaps+1, hardupsample, true,max,false);
            break;
		case unity:
		{
			for(int i=0;i<16;i++)
			{
				if(i==0)
				{
					 firtx[i]=1.0;
					 firrx[i]=1.0;
				}	 
				else
				{
					 firtx[i]=0.0;
					 firrx[i]=0.0;
				}
					 
			}	
			//set_filter_gain(firtx,ratio,16);
			//set_filter_gain(firrx,ratio,16);
			fmc_load_tx_filter(firrx,firtx, 16, hardupsample, true,max,false);
		}
		break;	
    }
    
	// For designing and output coef for FPGA FIR
    /*
			build_rrc_filter(firtx,0.35,129,8);
			FILE *fcoef=fopen("Interpol.coe","wb");
			max=set_filter_gain(firtx,1,129);
			fprintf(fcoef,"Radix = 10;\nCoefficient_Width = 16;\nCoefData=");
			for(int i=0;i<129;i++)
			{
				if(i!=128)
					fprintf(fcoef,"%d,\n",(short)(0x3FFF*firtx[i]/max));
				else
					fprintf(fcoef,"%d;",(short)(0x3FFF*firtx[i]/max));
			}
			fclose(fcoef);	
	*/		
		

}

void fmc_set_sr(long long int sr,bool tx)
{
	struct iio_channel *chn = NULL;
	if (m_ctx == NULL)
		return;
	if (!get_phy_chan(m_ctx, tx?TX:RX, 0, &chn))
	{
		fprintf(stderr,"Could not find channel\n");
		return;
	}
	wr_ch_lli(chn, "sampling_frequency", (long long int)sr);

    
}

long long int ComputeSR(long long int sr,int *adupsample,int *fpgaupsample,int *softupsample,bool tx)
{
	// 1 nominal SR, use interpol ad and last use x8 fpga interpol
	*adupsample=1;
	*fpgaupsample=1;
	
	long long int Symbolrate= sr;

	sr=sr*(*softupsample); // We know already that we decided to softupsample
	
	if(sr<MAX_ADC_CLK/8)
	{
		*fpgaupsample=8;
		sr*=8;
	}
	

	/*
	if(sr*8<MIN_ADC_CLK)
	{
		*fpgaupsample=8;
		sr*=8;
	}
	*/

	while((*adupsample<4)&&(sr*(*adupsample)<MIN_ADC_CLK))
	{
		(*adupsample)*=2;
	}
	
	sr=sr*(*adupsample);

	while(sr<MIN_ADC_CLK)
	{
		(*softupsample)*=2;
		sr*=2;	
	}
	return Symbolrate*(*softupsample);
}

int set_int_filter_rates( long long rate)
{
	int ret;
	struct iio_channel *chan;

	chan = iio_device_find_channel(m_tx, "voltage0", true);
	if (chan == NULL)
		return -ENODEV;
	ret = iio_channel_attr_write_longlong(chan, "sampling_frequency", rate);
	if (ret < 0)
		return ret;
}

int set_dec_filter_rates( long long rate)
{
	int ret;
	struct iio_channel *chan;
	
	chan = iio_device_find_channel(m_rx, "voltage0", false);
	if (chan == NULL)
		return -ENODEV;
	ret = iio_channel_attr_write_longlong(chan, "sampling_frequency", rate);
	if (ret < 0)
		return ret;
}

int set_int_dec_filter_rates( long long rate)
{
	set_int_filter_rates(rate);
	set_dec_filter_rates(rate);
}

int fmc_setppm(float ppm)
{
	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);
	long long FrequencyXO = 40e6 +40*ppm;
	 iio_device_attr_write_longlong(dev,"xo_correction",FrequencyXO);
	return 0;
}

void fmc_perfom_calibration(bool calibrate)
{

	if (m_ctx == NULL)
		return;
	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);

	
	if (calibrate)
	{
		
		iio_device_attr_write(dev, "calib_mode", "auto");
		
	}
	else
	{
		iio_device_attr_write(dev, "calib_mode", "manual_tx_quad");
		
	}
	
}

void fmc_perfom_rssi_calibration(bool calibrate)
{

	if (m_ctx == NULL)
		return;
	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);

	
	if (calibrate)
	{
		
		iio_device_attr_write(dev, "calib_mode", "rssi_gain_step");
		
	}
	else
	{
		iio_device_attr_write(dev, "calib_mode", "manual_tx_quad");
		
	}
	
}

void fmc_enable_tx_lo(void)
{
	struct iio_channel *chn = NULL;
	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);
	chn = iio_device_find_channel(dev, "altvoltage1", true);
	iio_channel_attr_write_bool(chn, "powerdown", false);
}

void fmc_disable_tx_lo(void)
{
	struct iio_channel *chn = NULL;
	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);
	chn = iio_device_find_channel(dev, "altvoltage1", true);
	iio_channel_attr_write_bool(chn, "powerdown", true);
}
void fmc_tx_ptt(bool Tx)
{
	struct iio_device *dev = NULL;
	dev = get_ad9361_phy(m_ctx);

	 
	if(Tx)
	{
		
		fmc_enable_tx_lo();
		
		iio_device_debug_attr_write(dev,"direct_reg_access","0x27 0x10");
	}	
	else
	{
		
		fmc_disable_tx_lo();	
		iio_device_debug_attr_write(dev,"direct_reg_access","0x27 0x00");
	}
	
		
}


// ********************** FASTLOCK FUNCTIONS **********************************
char fastlock_param_tx[8][255];
char fastlock_param_rx[8][255];

char* GetFastlockParam(int fastlock_profile_no,bool tx)
{
	
	struct iio_channel *chn = NULL;
	if(tx)
	{
		chn=iio_device_find_channel(m_dev, "altvoltage1", true);
		iio_channel_attr_write_longlong(chn, "fastlock_store", fastlock_profile_no);
		iio_channel_attr_write_longlong(chn, "fastlock_save", fastlock_profile_no);
		iio_channel_attr_read(chn,"fastlock_save",fastlock_param_tx[fastlock_profile_no],255);
		return fastlock_param_tx[fastlock_profile_no]; 
	}	
	else
	{
		chn=iio_device_find_channel(m_dev, "altvoltage0", true);
		iio_channel_attr_write_longlong(chn, "fastlock_store", fastlock_profile_no);
		iio_channel_attr_write_longlong(chn, "fastlock_save", fastlock_profile_no);
		iio_channel_attr_read(chn,"fastlock_save",fastlock_param_rx[fastlock_profile_no],255);
		return fastlock_param_rx[fastlock_profile_no]; 
	}
		
	
	
	
}

void SetFastlockParam(char *fastlockparam,int fastlock_profile_no,bool tx)
{
	struct iio_channel *chn = NULL;
	if(tx)
		chn=iio_device_find_channel(m_dev, "altvoltage1", true);
	else
	{
		chn=iio_device_find_channel(m_dev, "altvoltage0", true);
	}
	
	fastlockparam[0]='0'+fastlock_profile_no; // We overwrite the profile
	//fprintf(stderr,"FastProfile %d : %s\n",fastlock_profile_no,fastlockparam);
	iio_channel_attr_write(chn,"fastlock_load",fastlockparam);
	
}

void SetFastlockTune(int fastlock_profile_no,bool tx)
{
	struct iio_channel *chn = NULL;
	if(tx)
	{
		chn=iio_device_find_channel(m_dev, "altvoltage1", true);
	}	
	else
	{
		chn=iio_device_find_channel(m_dev, "altvoltage0", true);
	}
	iio_channel_attr_write_longlong(chn,"fastlock_recall",fastlock_profile_no);

}
// ********************** RSSI ******************************************
void InitRSSI()
{
	iio_device_debug_attr_write(m_dev, "direct_reg_access", "0x158 0x10"); // RSSI MODE Trigger
	iio_device_debug_attr_write(m_dev, "direct_reg_access", "0x15C 0x70"); // RSSI Duration 0
		
}

double GetRssi(bool tx)
{
		struct iio_channel *chn = NULL;
		if(tx)
		{
			chn=iio_device_find_channel(m_dev, "voltage0", true);
		}
		else
		{
			chn=iio_device_find_channel(m_dev, "voltage0", false);
		};
		iio_device_debug_attr_write(m_dev, "direct_reg_access", "0x158 0x30"); // RSSI Trigger
		double rssi=0;
		iio_channel_attr_read_double(chn,"rssi",&rssi);
		return rssi;
}

// ********************** DDS FUNCTIONS **********************************
void dds_enable(bool enable)
{

	iio_channel_attr_write_bool(iio_device_find_channel(m_tx, "altvoltage0",
                                      true), "raw", enable);
	iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage0",true), "scale", 0); 
	iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage2",true), "scale", 0); 
}

void dds_frequency(double frequency)
{
	int ret=0;

	ret=iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage0",true), "frequency", frequency);
	ret=iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage2",true), "frequency", frequency);
	ret=iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage0",true), "scale", 0.2238); //65535/4096/2
	ret=iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage2",true), "scale", 0.2238); //65535/4096/2									
	if(ret!=0) fprintf(stderr,"error %d\n",ret);
	ret=iio_channel_attr_write_double(iio_device_find_channel(m_tx, "altvoltage1",
                                        true), "scale", 0.0);					
	if(ret!=0) fprintf(stderr,"error %d\n",ret);														
}

// ********************** CHANNELS FUNCTIONS **********************************

static uint64_t _timestamp_ns(void)
{
	struct timespec tp;

	if (clock_gettime(CLOCK_REALTIME, &tp) != 0)
	{
		return (0);
	}

	return ((int64_t)tp.tv_sec * 1e9 + tp.tv_nsec);
}

void fmc_initchannel(uint32_t len, unsigned int nbBuffer,bool circular,bool blocking)
{
	//circular=false;
	//blocking=true
	iio_context_set_timeout(m_ctx, 0);
	
	//	printf("* Initializing AD9361 IIO streaming channels\n");
	FMC_ERROR(get_ad9361_stream_ch(m_ctx, TX, m_tx, 0, &m_tx0_i) && "TX chan i not found");
	FMC_ERROR(get_ad9361_stream_ch(m_ctx, TX, m_tx, 1, &m_tx0_q) && "TX chan q not found");

	//	printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(m_tx0_i);
	iio_channel_enable(m_tx0_q);

	fprintf(stderr, "Stream with %u buffers of %d samples\n", nbBuffer, len);
	// Change the size of the buffer
	m_max_len = len;
	if (m_txbuf)
	{
		iio_buffer_destroy(m_txbuf);
		m_txbuf = NULL;
		fprintf(stderr, "BIG ISSUE : detroying buffer\n");
	}
	m_txbuf = iio_device_create_buffer(m_tx, len, circular);
	if(m_txbuf==NULL) 
	{
		fprintf(stderr,"Could not allocate iio mem\n");
		exit(1);
	}
	iio_device_set_kernel_buffers_count(m_tx, nbBuffer);
	
	iio_buffer_set_blocking_mode(m_txbuf, blocking);
	
	if(((struct iio_buffer*)m_txbuf)->dev_is_high_speed)
		fprintf(stderr,"Buffer is HighSpeed\n");
	else
	{
		fprintf(stderr,"Buffer is LowSpeed\n");
	}
	
	
	sfcmplx *buffpluto = (sfcmplx *)iio_buffer_first(m_txbuf, m_tx0_i);
	memset(buffpluto,0,m_max_len*sizeof(sfcmplx));
	m_offset = 0;
}

void fmc_initchannelrx(uint32_t len, unsigned int nbBuffer)
{
	iio_context_set_timeout(m_ctx, 0);
	
	//	printf("* Initializing AD9361 IIO streaming channels\n");
	FMC_ERROR(get_ad9361_stream_ch(m_ctx, RX, m_rx, 0, &m_rx0_i) && "RX chan i not found");
	FMC_ERROR(get_ad9361_stream_ch(m_ctx, RX, m_rx, 1, &m_rx0_q) && "RX chan q not found");

	//	printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(m_rx0_i);
	iio_channel_enable(m_rx0_q);

	fprintf(stderr, "Rcv Stream with %u buffers of %d bytes\n", nbBuffer, len);
	// Change the size of the buffer
	m_max_len = len;
	if (m_rxbuf)
	{
		iio_buffer_destroy(m_rxbuf);
		m_rxbuf = NULL;
		fprintf(stderr, "BIG ISSUE : detroying buffer\n");
	}
	m_rxbuf = iio_device_create_buffer(m_rx, len, false);
	if(m_rxbuf==NULL) 
	{
		fprintf(stderr,"Could not allocate iio mem\n");
		exit(1);
	}
	iio_device_set_kernel_buffers_count(m_rx, nbBuffer);
	
	iio_buffer_set_blocking_mode(m_rxbuf, true);
	
	if(((struct iio_buffer*)m_rxbuf)->dev_is_high_speed)
		fprintf(stderr,"Buffer is HighSpeed\n");
	else
	{
		fprintf(stderr,"Buffer is LowSpeed\n");
	}
	
	int ret = iio_device_reg_write(m_rx, 0x80000088, 0x4);
	if (ret) {
		fprintf(stderr, "Failed to clearn DMA status register: %s\n",
				strerror(-ret));
	}

	/*sfcmplx *buffpluto = (sfcmplx *)iio_buffer_first(m_rxbuf, m_rx0_i);
	memset(buffpluto,0,m_max_len*sizeof(sfcmplx));
	*/
	m_offset = 0;
}

void fmc_tx_samples(sfcmplx *s, int len)
{

	if (len == 0)
	{
		fprintf(stderr, "Len = 0 \n");
		return;
	}
	// Get position of first sample in the buffer
	
	sfcmplx *s_b = (sfcmplx *)iio_buffer_first(m_txbuf, m_tx0_i);

	if ((m_offset + len) > m_max_len)
	{
		int l = m_max_len - m_offset;
		sfcmplx *b = (sfcmplx *)&s_b[m_offset];

		memcpy(b, s, sizeof(sfcmplx) * l);
		int n = iio_buffer_push(m_txbuf);
		if (n != (m_max_len << 2))
			fprintf(stderr, "buffer push error %d/%u len %d\n", n, (m_max_len), len);
		s = &s[l];
		len = len - l;
		m_offset = 0;
	}
	// Calculate the offset for the begining of the next update
	sfcmplx *b = (sfcmplx *)&s_b[m_offset];
	// Copy the Samples into the buffer
	memcpy(b, s, sizeof(sfcmplx) * len); // Fixme : Inspect here an issue with core dump !
	m_offset += len;
}

void fmc_tx_samples2(sfcmplx *s, int len,int timeout)
{
	//timeout=0;
	if(iio_buffer_get_poll_fd(m_txbuf)<0)
	{
			fprintf(stderr,"Full\n");
	}
	sfcmplx *buffpluto = (sfcmplx *)iio_buffer_first(m_txbuf, m_tx0_i);
	memcpy(buffpluto,s,len*sizeof(sfcmplx));
	
	//fprintf(stderr,"Buffer %d\n",len);	
		for (int i=0;i<len;i++)
		{
			*buffpluto=s[i];
			buffpluto++;
			
		}
		if(iio_buffer_push_partial(m_txbuf,len)<len)
		{
			if(timeout)
			{
				usleep(timeout);
			}
		}
		uint32_t val=0;
		int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
		if(val&1)
		{
			fprintf(stderr, "!underflow!");fflush(stderr);
			iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits	
		}	
}
void fmc_tx_samples_upsample(sfcmplx *s, int len,int upsample) //Effective but not generic : len*upsample should be exact size of buffer
{

	sfcmplx *buffpluto = (sfcmplx *)iio_buffer_first(m_txbuf, m_tx0_i);
	memset(buffpluto,0,m_max_len*sizeof(sfcmplx));

		
		for (int i=0;i<len;i++)
		{
			*buffpluto=s[i];
			buffpluto+=upsample;
			
		}
		uint64_t t0=_timestamp_ns();
		iio_buffer_push(m_txbuf);	
			fprintf(stderr,"Tx = %lu\n",_timestamp_ns()-t0);
		
}

void fmc_tx_samples_partial(sfcmplx *s, int len, int NbPartialBuffer)
{
	/*
	static uint64_t time_init = _timestamp_ns();
	uint64_t t0 = _timestamp_ns();
	static uint64_t lasttimecall = _timestamp_ns();
	uint64_t t1 = _timestamp_ns();
	*/
	static int CountPartial = 0;
	
	void *ptrtowritre = iio_buffer_start(m_txbuf) + CountPartial * sizeof(sfcmplx) * len;
	memcpy(ptrtowritre, s, sizeof(sfcmplx) * len);
	if (CountPartial == NbPartialBuffer - 1)
	{
		CountPartial = 0;
		int written = iio_buffer_push(m_txbuf);
		if (written < 0)
		{
			fprintf(stderr, "Overflow :");
			//fprintf(stderr, " Tx =(%d/%d) %llu : %llu\n", len, written, (t0 - lasttimecall) / 1000000, (_timestamp_ns() - t0) / 1000000);
			//written = iio_buffer_push(m_txbuf);
		}
		uint32_t val=0;
		int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
		if(val&1)
		{
			fprintf(stderr, "!underflow!");fflush(stderr);
			iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits	
		}
				
		//lasttimecall = t1;
	}
	else
	{
		CountPartial++;
	}
	
}

ssize_t fmc_rx_samples(short *iarray, short *qarray)
{
	ssize_t nsamples_rx = iio_buffer_refill(m_rxbuf)/(2*sizeof(short));
	if (nsamples_rx < 0)
	{
		fprintf(stderr,"Error refilling Rx buf %d\n",(int) nsamples_rx);
	}	
	//short *Rx = iio_buffer_start(m_rxbuf);
	short *Rx =(short*) iio_buffer_first(m_rxbuf, m_rx0_i);
	for(size_t i=0;i<nsamples_rx;i++)
	{
		iarray[i]=Rx[2*i];
		qarray[i]=Rx[2*i+1];
	}
	return nsamples_rx;
	
}

ssize_t fmc_direct_rx_samples(short **RxBuffer)
{
	ssize_t nsamples_rx = iio_buffer_refill(m_rxbuf)/(2*sizeof(short));

	if (nsamples_rx < 0)
	{
		fprintf(stderr,"Error refilling Rx buf %d\n",(int) nsamples_rx);
	}	
	
	// *RxBuffer =(short*) iio_buffer_start(m_rxbuf);
	*RxBuffer =(short*) iio_buffer_first(m_rxbuf, m_rx0_i);
	
	uint32_t val=0;
	//https://wiki.analog.com/resources/fpga/docs/hdl/regmap
	int ret = iio_device_reg_read(m_rx, 0x80000088, &val);
	if(val&4)
	{
		fprintf(stderr, "!");fflush(stderr);
		iio_device_reg_write(m_rx, 0x80000088, val); // Clear bits	
	}
	return nsamples_rx;
	
}

ssize_t fmc_rx_samples_float(float *RxFBuffer)
{
	short *RxBuffer;
	ssize_t NbRx=fmc_direct_rx_samples(&RxBuffer);
	for(ssize_t i=0;i<NbRx*2;i++)
	{
		
		RxFBuffer[i]=(float)((float)RxBuffer[i]/(float)(0x3FFF));
	}
	return NbRx*2;
	
}

/*You can find the related information in UG 570 AD9361 Reference Manual on page 18 (Lock Detector section).
Also, you can take a look at register 0x24A in UG 671 AD9361 Register Map Reference Manual.
20us is based on a measurement.*/
