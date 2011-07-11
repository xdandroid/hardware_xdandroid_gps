#include <stdio.h>
#include <stdlib.h>
#include <librpc/rpc/rpc.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <librpc/rpc/rpc_router_ioctl.h>
#include <pthread.h>
#include <hardware/gps.h>

#if defined(ANDROID)
#define LOG_NDEBUG 1
#define  LOG_TAG  "gps_rpc"
#include <cutils/log.h>
#include <cutils/properties.h>
#else
#define LOGV(fmt, x...) printf(fmt, ##x)
#define LOGI LOGV
#define LOGE LOGV
#endif

typedef struct registered_server_struct {
	/* MUST BE AT OFFSET ZERO!  The client code assumes this when it overwrites
	 * the XDR for server entries which represent a callback client.  Those
	 * server entries do not have their own XDRs.
	 */
	XDR *xdr;
	/* Because the xdr is NULL for callback clients (as opposed to true
	 * servers), we keep track of the program number and version number in this
	 * structure as well.
	 */
	rpcprog_t x_prog; /* program number */
	rpcvers_t x_vers; /* program version */

	int active;
	struct registered_server_struct *next;
	SVCXPRT *xprt;
	__dispatch_fn_t dispatch;
} registered_server;

struct SVCXPRT {
	fd_set fdset;
	int max_fd;
	pthread_attr_t thread_attr;
	pthread_t  svc_thread;
	pthread_mutexattr_t lock_attr;
	pthread_mutex_t lock;
	registered_server *servers;
	volatile int num_servers;
};



#define SEND_VAL(x) do { \
	val=x;\
	XDR_SEND_UINT32(clnt, &val);\
} while(0);

#define CLNT_CALL_CAST(a, b, c, d, e, f, g) clnt_call(a, b, (xdrproc_t) c, (caddr_t) d, (xdrproc_t) e, (caddr_t) f, g)

static uint32_t client_IDs[16];//highest known value is 0xb
static uint32_t can_send=1; //To prevent from sending get_position when EVENT_END hasn't been received

struct params {
	uint32_t *data;
	int length;
};
static struct CLIENT *_clnt;

static bool_t xdr_args(XDR *clnt, struct params *par) {
	int i;
	uint32_t val=0;
	for(i=0;par->length>i;++i)
		SEND_VAL(par->data[i]);
	return 1;
}

static bool_t xdr_result_int(XDR *clnt, uint32_t *result) {
	XDR_RECV_UINT32(clnt, result);
	return 1;
}

static struct timeval timeout;
static enum {
	A5225,
	A6125,
	INVALID_AMSS,
} amss = INVALID_AMSS;

static int pdsm_client_init(struct CLIENT *clnt, int client) {
	struct params par;
	uint32_t res;
	uint32_t data;
	par.data = &data;
	par.length=1;
	par.data[0]=client;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x2 : 0x3, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_init(%x) failed\n", client);
		exit(-1);
	}
	LOGV("pdsm_client_init(%x)=%x\n", client, res);
	client_IDs[client]=res;
	return 0;
}

int pdsm_atl_l2_proxy_reg(struct CLIENT *clnt, int val0, int val1, int val2) {
	struct params par;
	uint32_t res;
	uint32_t data[3];
	par.data = data;
	par.length=3;
	par.data[0]=val0;
	par.data[1]=val1;
	par.data[2]=val2;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x3 : 0x4, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_atl_l2_proxy_reg(%x, %x, %x) failed\n", par.data[0], par.data[1], par.data[2]);
		exit(-1);
	}
	LOGV("pdsm_atl_l2_proxy_reg(%x, %x, %x)=%x\n", par.data[0], par.data[1], par.data[2], res);
	return res;
}

int pdsm_atl_dns_proxy_reg(struct CLIENT *clnt, int val0, int val1) {
	struct params par;
	uint32_t res;
	uint32_t data[2];
	par.data = data;
	par.length=2;
	par.data[0]=val0;
	par.data[1]=val1;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x6 : 0x7 , xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_atl_dns_proxy_reg(%x, %x) failed\n", par.data[0], par.data[1]);
		exit(-1);
	}
	LOGV("pdsm_atl_dns_proxy(%x, %x)=%x\n", par.data[0], par.data[1], res);
	return res;
}

int pdsm_client_pd_reg(struct CLIENT *clnt, int client, int val0, int val1, int val2, int val3, int val4) {
	struct params par;
	uint32_t res;
	uint32_t data[6];
	par.data = data;
	par.length=6;
	par.data[0]=client_IDs[client];
	par.data[1]=val0;
	par.data[2]=val1;
	par.data[3]=val2;
	par.data[4]=val3;
	par.data[5]=val4;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x4 : 0x5, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_pd_reg(%d, %d, %d, %d, %d, %d) failed\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5]);
		exit(-1);
	}
	LOGV("pdsm_client_pd_reg(%d, %d, %d, %d, %d, %d)=%d\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5], res);
	return res;
}

int pdsm_client_pa_reg(struct CLIENT *clnt, int client, int val0, int val1, int val2, int val3, int val4) {
	struct params par;
	uint32_t res;
	uint32_t data[6];
	par.data = data;
	par.length=6;
	par.data[0]=client_IDs[client];
	par.data[1]=val0;
	par.data[2]=val1;
	par.data[3]=val2;
	par.data[4]=val3;
	par.data[5]=val4;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x5 : 0x6, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_pa_reg(%d, %d, %d, %d, %d, %d) failed\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5]);
		exit(-1);
	}
	LOGV("pdsm_client_pa_reg(%d, %d, %d, %d, %d, %d)=%d\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5], res);
	return res;
}

int pdsm_client_lcs_reg(struct CLIENT *clnt, int client, int val0, int val1, int val2, int val3, int val4) {
	struct params par;
	uint32_t res;
	uint32_t data[6];
	par.data = data;
	par.length=6;
	par.data[0]=client_IDs[client];
	par.data[1]=val0;
	par.data[2]=val1;
	par.data[3]=val2;
	par.data[4]=val3;
	par.data[5]=val4;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x6 : 0x7, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_lcs_reg(%d, %d, %d, %d, %d, %d) failed\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5]);
		exit(-1);
	}
	LOGV("pdsm_client_lcs_reg(%d, %d, %d, %d, %d, %d)=%d\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5], res);
	return res;
}

int pdsm_client_ext_status_reg(struct CLIENT *clnt, int client, int val0, int val1, int val2, int val3, int val4) {
	struct params par;
	uint32_t res;
	uint32_t data[6];
	par.data = data;
	par.length=6;
	par.data[0]=client_IDs[client];
	par.data[1]=val0;
	par.data[2]=val1;
	par.data[3]=val2;
	par.data[4]=val3;
	par.data[5]=val4;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x8 : 0x9, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_ext_status_reg(%d, %d, %d, %d, %d, %d) failed\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5]);
		exit(-1);
	}

	LOGV("pdsm_client_ext_status_reg(%d, %d, %d, %d, %d, %d)=%d\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5], res);
	return res;
}

int pdsm_client_xtra_reg(struct CLIENT *clnt, int client, int val0, int val1, int val2, int val3, int val4) {
	struct params par;
	uint32_t res;
	uint32_t data[6];
	par.data = data;
	par.length=6;
	par.data[0]=client_IDs[client];
	par.data[1]=val0;
	par.data[2]=val1;
	par.data[3]=val2;
	par.data[4]=val3;
	par.data[5]=val4;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x7 :0x8, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_xtra_reg(%d, %d, %d, %d, %d, %d) failed\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5]);
		exit(-1);
	}

	LOGV("pdsm_client_xtra_reg(%d, %d, %d, %d, %d, %d)=%d\n", par.data[0], par.data[1], par.data[2], par.data[3], par.data[4], par.data[5], res);
	return res;
}

int pdsm_client_act(struct CLIENT *clnt, int client) {
	struct params par;
	uint32_t res;
	uint32_t data;
	par.data = &data;
	par.length=1;
	par.data[0]=client_IDs[client];
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0x9 : 0xa, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_act(%d) failed\n", par.data[0]);
		exit(-1);
	}

	LOGV("pdsm_client_act(%d)=%d\n", par.data[0], res);
	return res;
}

int pdsm_client_get_position(struct CLIENT *clnt, int val0, int val1, int val2, int val3, int val4, int val5, int val6, int val7, int val8, int val9, int val10, int val11, int val12, int val13, int val14, int val15, int val16, int val17, int val18, int val19, int val20, int val21, int val22, int val23, int val24, int val25, int val26, int val27) {
	struct params par;
	uint32_t res;
	uint32_t data[28];
	par.data = data;
	par.length=28;
	par.data[0]=val0;
	par.data[1]=val1;
	par.data[2]=val2;
	par.data[3]=val3;
	par.data[4]=val4;
	par.data[5]=val5;
	par.data[6]=val6;
	par.data[7]=val7;
	par.data[8]=val8;
	par.data[9]=val9;
	par.data[10]=val10;
	par.data[11]=val11;
	par.data[12]=val12;
	par.data[13]=val13;
	par.data[14]=val14;
	par.data[15]=val15;
	par.data[16]=val16;
	par.data[17]=val17;
	par.data[18]=val18;
	par.data[19]=val19;
	par.data[20]=val20;
	par.data[21]=val21;
	par.data[22]=val22;
	par.data[23]=val23;
	par.data[24]=val24;
	par.data[25]=val25;
	par.data[26]=val26;
	par.data[27]=val27;
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0xb : 0xc, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_get_position() failed\n");
		exit(-1);
	}
	LOGV("pdsm_client_get_position()=%d\n", res);
	return res;
}

enum pdsm_pd_events {
	PDSM_PD_EVENT_POS = 0x1,
	PDSM_PD_EVENT_VELOCITY = 0x2,
	PDSM_PD_EVENT_HEIGHT = 0x4,
	PDSM_PD_EVENT_DONE = 0x8,
	PDSM_PD_EVENT_END = 0x10,
	PDSM_PD_EVENT_BEGIN = 0x20,
	PDSM_PD_EVENT_COMM_BEGIN = 0x40,
	PDSM_PD_EVENT_COMM_CONNECTED = 0x80,
	PDSM_PD_EVENT_COMM_DONE = 0x200,
	PDSM_PD_EVENT_GPS_BEGIN = 0x4000,
	PDSM_PD_EVENT_GPS_DONE = 0x8000,
	PDSM_PD_EVENT_UPDATE_FAIL = 0x1000000,
};

//From gps_msm7k.c
extern void update_gps_status(GpsStatusValue val);
extern void update_gps_svstatus(GpsSvStatus *val);
extern void update_gps_location(GpsLocation *fix);

void dispatch_pdsm_pd(uint32_t *data) {
	uint32_t event=ntohl(data[2]);
	LOGV("dispatch_pdsm_pd(): event=0x%x",event);
	if(event&PDSM_PD_EVENT_GPS_BEGIN) {
		//Navigation started.
		update_gps_status(GPS_STATUS_SESSION_BEGIN);
	}
	if(event&PDSM_PD_EVENT_GPS_DONE) {
		//Navigation ended (times out circa 10seconds ater last get_pos)
		update_gps_status(GPS_STATUS_SESSION_END);
	}
	GpsLocation fix;
	memset(&fix,0,sizeof(GpsLocation));
	if(event&PDSM_PD_EVENT_POS) {

		GpsSvStatus ret;
		int i;
		ret.num_svs=ntohl(data[82]) & 0x1F;

		for(i=0;i<ret.num_svs;++i) {
			ret.sv_list[i].prn=ntohl(data[83+3*i]);
			ret.sv_list[i].elevation=ntohl(data[83+3*i+1]);
			ret.sv_list[i].azimuth=ntohl(data[83+3*i+2])/100;
			ret.sv_list[i].snr=ntohl(data[83+3*i+2])%100;
		}
		ret.used_in_fix_mask=ntohl(data[77]);
		update_gps_svstatus(&ret);

		fix.timestamp = ntohl(data[8]);
		if (!fix.timestamp) return;

		// convert gps time to epoch time ms
		fix.timestamp += 315964800; // 1/1/1970 to 1/6/1980
		fix.timestamp *= 1000; //ms

		fix.flags |= GPS_LOCATION_HAS_LAT_LONG;

		if (ntohl(data[75])) {
			fix.flags |= GPS_LOCATION_HAS_ACCURACY;
			fix.accuracy = (float)ntohl(data[75]) / 10.0f;
		}

		union {
			struct {
				uint32_t lowPart;
				int32_t highPart;
			};
			int64_t int64Part;
		} latitude, longitude;

		latitude.lowPart = ntohl(data[61]);
		latitude.highPart = ntohl(data[60]);
		longitude.lowPart = ntohl(data[63]);
		longitude.highPart = ntohl(data[62]);
		fix.latitude = (double)latitude.int64Part / 1.0E8;
		fix.longitude = (double)longitude.int64Part / 1.0E8;
	}
	if (event&PDSM_PD_EVENT_VELOCITY)
	{
		fix.flags |= GPS_LOCATION_HAS_SPEED|GPS_LOCATION_HAS_BEARING;
		fix.speed = (float)ntohl(data[66]) / 10.0f / 3.6f; // convert kp/h to m/s
		fix.bearing = (float)ntohl(data[67]) / 10.0f;
	}
	if (event&PDSM_PD_EVENT_HEIGHT)
	{
		fix.flags |= GPS_LOCATION_HAS_ALTITUDE;
		fix.altitude = (double)ntohl(data[64]) / 10.0f;
	}
	/*
	We get frequent PDSM_PD_EVENT_HEIGHTs when not locked.
	This should never happen - there is no way to know height
	(3D fix,4+ sats) when we don't know position (2D fix, 3 sats)
	without a barometer - there is no known mobile phone HW with
	a barometric sensor
	As a result - don't update position unless we have lat/long
	*/
	if (fix.flags & GPS_LOCATION_HAS_LAT_LONG)
	{
		LOGV("updating GPS location");
		update_gps_location(&fix);
	}
	if(event&PDSM_PD_EVENT_DONE)
		can_send=1;
}

void dispatch_pdsm_ext(uint32_t *data) {

	GpsSvStatus ret;
	int i;

	LOGV("dispatch_pdsm_ext()");

	ret.num_svs=ntohl(data[7]);
	for(i=0;i<ret.num_svs;++i) {
		ret.sv_list[i].prn=ntohl(data[101+12*i]);
		ret.sv_list[i].elevation=ntohl(data[101+12*i+4]);
		ret.sv_list[i].azimuth=ntohl(data[101+12*i+3]);
		ret.sv_list[i].snr=(float)ntohl(data[101+12*i+1])/10.0f;
	}

	ret.used_in_fix_mask=ntohl(data[8]);
	if (ret.num_svs) {
		update_gps_svstatus(&ret);
	}

}

void dispatch_pdsm(uint32_t *data) {
	uint32_t procid=ntohl(data[5]);
	if(procid==1)
		dispatch_pdsm_pd(&(data[10]));
	else if(procid==4)
		dispatch_pdsm_ext(&(data[10]));
	else
		LOGE("dispatch_pdsm() received unknown procid: %d",procid);
}

void dispatch_atl(uint32_t *data) {
	// No clue what happens here.
}

void dispatch(struct svc_req* a, registered_server* svc) {
	int i;
	uint32_t *data = (uint32_t*)svc->xdr->in_msg;
	uint32_t result=0;
	uint32_t svid=ntohl(data[3]);
	LOGV("received some kind of event\n");
	for(i=0;i< svc->xdr->in_len/4;++i) {
		LOGV("%08x ", ntohl(data[i]));
	}
	LOGV("\n");
	for(i=0;i< svc->xdr->in_len/4;++i) {
		LOGV("%010d ", ntohl(data[i]));
	}
	LOGV("\n");
	if(svid==0x3100005b) {
		dispatch_pdsm(data);
	} else if(svid==0x3100001d) {
		dispatch_atl(data);
	} else {
		LOGE("%s: dispatch for unknown server id=0x%08x\n", __func__, svid);
	}
	//ACK
	svc_sendreply((struct SVCXPRT*) svc, (xdrproc_t) xdr_int, (caddr_t) &result);
}

int pdsm_client_end_session(struct CLIENT *clnt, int id, int client) {
	struct params par;
	uint32_t res;
	uint32_t data[4];
	par.data = data;
	par.length=4;
	par.data[0]=id;
	par.data[1]=0;
	par.data[2]=0;
	par.data[3]=client_IDs[client];
	if(CLNT_CALL_CAST(clnt, amss==A6125 ? 0xd : 0xe, xdr_args, &par, xdr_result_int, &res, timeout)) {
		LOGV("pdsm_client_end_session(%x, 0, 0, %x) failed\n", id, client_IDs[client]);
		exit(-1);
	}

	LOGV("pdsm_client_end_session(%x, 0, 0, %x)=%x\n", id, client_IDs[client], res);
	return 0;
}

static int init_gps6125() {
	struct CLIENT *clnt=clnt_create(NULL, 0x3000005B, 0, NULL);
	struct CLIENT *clnt_atl=clnt_create(NULL, 0x3000001D, 0, NULL);
	int i;
	_clnt=clnt;
	SVCXPRT *svc=svcrtr_create();
	xprt_register(svc);
	svc_register(svc, 0x3100005b, 0xb93145f7, (__dispatch_fn_t) dispatch,0);
	svc_register(svc, 0x3100005b, 0, (__dispatch_fn_t) dispatch,0);
	svc_register(svc, 0x3100001d, 0/*xb93145f7*/, (__dispatch_fn_t) dispatch,0);
	if(!clnt) {
		LOGE("Failed creating client\n");
		return -1;
	}
	if(!svc) {
		LOGE("Failed creating server\n");
		return -2;
	}

	pdsm_client_init(clnt, 2);
	pdsm_client_pd_reg(clnt, 2, 0, 0, 0, 0xF3F0FFFF, 0);
	pdsm_client_ext_status_reg(clnt, 2, 0, 0, 0, 0x4, 0);
	pdsm_client_act(clnt, 2);
	pdsm_client_pa_reg(clnt, 2, 0, 2, 0, 0x7ffefe0, 0);
	pdsm_client_init(clnt, 0xb);
	pdsm_client_xtra_reg(clnt, 0xb, 0, 3, 0, 7, 0);
	pdsm_client_act(clnt, 0xb);
	pdsm_atl_l2_proxy_reg(clnt_atl, 1,0,0);
	pdsm_atl_dns_proxy_reg(clnt_atl, 1,0);
	pdsm_client_init(clnt, 4);
	pdsm_client_lcs_reg(clnt, 4, 0,0,0,0x3f0, 0);
	pdsm_client_act(clnt, 4);

	return 0;
}

static int init_gps5225() {
	struct CLIENT *clnt=clnt_create(NULL, 0x3000005B, 0, NULL);
	struct CLIENT *clnt_atl=clnt_create(NULL, 0x3000001D, 0, NULL);
	int i;
	_clnt=clnt;
	SVCXPRT *svc=svcrtr_create();
	xprt_register(svc);
	svc_register(svc, 0x3100005b, 0xb93145f7, (__dispatch_fn_t) dispatch,0);
	svc_register(svc, 0x3100005b, 0, (__dispatch_fn_t) dispatch,0);
	svc_register(svc, 0x3100001d, 0/*xb93145f7*/, (__dispatch_fn_t) dispatch,0);
	if(!clnt) {
		LOGE("Failed creating client\n");
		return -1;
	}
	if(!svc) {
		LOGE("Failed creating server\n");
		return -2;
	}

	pdsm_client_init(clnt, 2);
	pdsm_client_pd_reg(clnt, 2, 0, 0, 0, 0xF310FFFF, 0xffffffff);
	pdsm_client_ext_status_reg(clnt, 2, 0, 1, 0, 4, 0xffffffff);
	pdsm_client_act(clnt, 2);
	pdsm_client_pa_reg(clnt, 2, 0, 2, 0, 0x003fefe0, 0xffffffff);
	pdsm_client_init(clnt, 0xb);
	pdsm_client_xtra_reg(clnt, 0xb, 0, 3, 0, 7, 0xffffffff);
	pdsm_client_act(clnt, 0xb);
	pdsm_atl_l2_proxy_reg(clnt_atl, 1, 4, 5);
	pdsm_atl_dns_proxy_reg(clnt_atl, 1, 6);
	pdsm_client_init(clnt, 0x4);
	pdsm_client_lcs_reg(clnt, 0x4, 0, 7, 0, 0x30f, 8);
	pdsm_client_act(clnt, 0x4);

	return 0;
}

static int init_gps_htc_hw(void) {
	int fd;
	char buf[4] = {};
	fd = open("/sys/class/htc_hw/amss", O_RDONLY);
	if (fd < 0)
		return fd;

	read(fd, buf, 4);
	close(fd);

	if(!strncmp(buf, "6125", 4))
		amss = A6125;
	else if((!strncmp(buf, "5225", 4)) || (!strncmp(buf, "6150", 4)))
		amss = A5225;
	else
		amss = A6125; //Fallback to 6125 ATM

	switch (amss) {
		case A6125:
			return init_gps6125();
		case A5225:
			return init_gps5225();
		default:
			break;
	}
	return -1;
}

#if defined(ANDROID)
static int init_gps_android_props(void) {
	char buf[PROPERTY_VALUE_MAX];
	char *tok;
	int version;

	if (!property_get("gsm.version.baseband", buf, "0.0.0.0"))
		return -1;
	tok = strtok(buf, ".");
	version = 100 * atoi(tok);
	tok = strtok(NULL, ".");

	//skip second number
	if (!tok)
		return -1;
	//bail out if we could not parse amss version
	tok = strtok(NULL, ".");
	if (!tok)
		return -1;

	version += atoi(tok);

	LOGI("%s: amss version=%d\n", __func__, version);
	switch (version) {
		case 5225:
		case 6150:
			amss = A5225;
			return init_gps5225();

		case 6125:
		default:
			amss = A6125;
			return init_gps6125();
	}

	return -1;
}
#endif

int init_gps_rpc() {
	int rc;
	rc = init_gps_htc_hw();
	if (!rc)
		return 0;

#if defined(ANDROID)
	return init_gps_android_props();
#endif

	return -1;
}

void gps_get_position() {
	int i;
	for (i = 0; i <= 5000 && !can_send; i += 100) {
		usleep(100000);
	}
	LOGV("%s: waited %dms for can_send\n", __func__, i);
	can_send = 0;
	pdsm_client_get_position(_clnt, 0, 0, 1, 1, 1, 0x3B9AC9FF, 1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,1,32,2,client_IDs[2]);
}

void exit_gps_rpc() {
	if(amss==A6125)
		pdsm_client_end_session(_clnt, 0, 2);
	//5225 doesn't seem to like end_session ?
	//Bah it ends session on itself after 10seconds.
}
