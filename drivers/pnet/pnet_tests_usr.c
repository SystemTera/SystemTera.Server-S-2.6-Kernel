#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/times.h>

#define TEST1_DO	1
#define TEST2_DO	0
#define TEST3_DO	0


#define BUFFER_LEN		1000000
#define PACKET_DATA_LEN		12
#define DMA_BUFFER_NUM		8

unsigned char wr_buffer[BUFFER_LEN];
unsigned char rd_buffer[BUFFER_LEN];

int spi_fd;
fd_set readers, writers;

int test1(void);
int test1_check_immediate(int bytes);
int test2(void);
int test3(void);
int test4(void);
int test5(void);
int test6(void);

void ndelay(long ns);
void print_stats(void);
void clock_start(void);
void clock_stop(void);
float clock_get_sec(void);

int main(int argc, char ** argv)
{
	int bytes;
	int ok;	
	printf("\nNetsilicon SPI tests\n\n");

	if (argc!=3) {
		printf("Usage: %s device bytes\n",argv[0]);
		return -1;
	}


	if ((spi_fd=open(argv[1], O_RDWR)) == -1) {
		printf("could not open %s\n",argv[1]);
		return -1;
	}
	
	bytes=atoi(argv[2]);
	if (bytes>BUFFER_LEN) {
		printf("bytes > %d not allowed\n",BUFFER_LEN);
		return -1;
	}

	
	#if (TEST1_DO==1)
	ok=test1_check_immediate(bytes);
	print_stats();
	if (!ok) goto error;
	#endif

	#if (TEST2_DO==1)
	ok=test2();
	print_stats();
	if (!ok) goto error;
	#endif

	#if (TEST3_DO==1)
	ok=test3();
	print_stats();
	if (!ok) goto error;
	#endif

	close(spi_fd);
	return 0;

error:
	close(spi_fd);
	return 1;
	
}


#define	TEST1_BYTES		1000000
int test1(void)
{
	int wr_sum,wr;
	int rd_sum,rd;
	int i,ok;
	for (i=0;i<TEST1_BYTES;i++) {
		wr_buffer[i]=(unsigned char) i;
		rd_buffer[i]=0;	
	}	
	rd_sum=0;
	wr_sum=0;
	printf("\nTEST1: Transmit %d Bytes bidirectional (loopback)\n", TEST1_BYTES);
	printf("TEST1: Start\n");
	clock_start();
	FD_ZERO(&readers);
	FD_ZERO(&writers);
	while (wr_sum<TEST1_BYTES || rd_sum<TEST1_BYTES) {
		FD_SET(spi_fd, &readers);
		FD_SET(spi_fd, &writers);
		select(spi_fd+1,&readers,&writers,NULL,NULL);
		if (FD_ISSET(spi_fd,&writers) && wr_sum<TEST1_BYTES) {
			wr=write(spi_fd,&wr_buffer[wr_sum],TEST1_BYTES-wr_sum);
			if (wr>0) wr_sum+=wr;
			else { printf("Disconnected\n"); return 0; }
		}
		if (FD_ISSET(spi_fd,&readers) && rd_sum<TEST1_BYTES) {				
			rd=read(spi_fd,&rd_buffer[rd_sum],TEST1_BYTES-rd_sum);
			if (rd>0) rd_sum+=rd;
			else { printf("Disconnected\n"); return 0; }
		}				
		//printf("wr:%d rd:%d\n",wr_sum,rd_sum);			
	}	
	clock_stop();
	printf("TEST1: End\n");
	ok=1;
	for (i=0;i<TEST1_BYTES;i++) {
		if (wr_buffer[i]!=rd_buffer[i]) ok=0;
	}			
	if(ok) 
		printf("TEST1: Successfull\n");
	else
		printf("TEST1: ERROR: received data doesnt match transmitted data\n");
	printf("TEST1: Time: %f sec\n",clock_get_sec());	
	return ok;
}	


int test1_check_immediate(int bytes)
{
	int wr_sum,wr;
	int rd_sum,rd;
	int i,j, ok;

	for (i=0;i<bytes;i++) {
		wr_buffer[i]=(unsigned char) i;
		rd_buffer[i]=0;	
	}	
	rd_sum=0;
	wr_sum=0;
	printf("\nTEST1: Transmit %d Bytes bidirectional (loopback)\n", bytes);
	printf("TEST1: Start\n");
	clock_start();
	FD_ZERO(&readers);
	FD_ZERO(&writers);
	while (wr_sum<bytes || rd_sum<bytes) {
		FD_SET(spi_fd, &readers);
		FD_SET(spi_fd, &writers);
		select(spi_fd+1,&readers,&writers,NULL,NULL);
		if (FD_ISSET(spi_fd,&writers) && wr_sum<bytes) {
			wr=write(spi_fd,&wr_buffer[wr_sum],bytes-wr_sum);
			if (wr>0) wr_sum+=wr;
			else { printf("Disconnected\n"); return 0; }
		}
		if (FD_ISSET(spi_fd,&readers) && rd_sum<bytes) {				
			rd=read(spi_fd,&rd_buffer[rd_sum],bytes-rd_sum);
			if (rd>0) {
				//sofort überprüfen
				for (i=0;i<rd;i++) {
					if (rd_buffer[rd_sum+i]!=wr_buffer[rd_sum+i]) {
						printf("TEST1: ERROR: received data doesnt match transmitted data\n");
						printf("TEST1: ERROR: at pos: %d, rd_sum: %d, rd:%d \n",rd_sum+i, rd_sum, rd);

						printf("data before rdsum:\n");
						for (j=rd_sum-32;j<rd_sum;j++) {
							if (j>0) {
								printf("%2.2x ",rd_buffer[j]);
								if (j%16==0) printf("\n");
							}
						}
						printf("\n\n");
						for (j=rd_sum;j<rd_sum+rd;j++) {
							printf("%2.2x ",rd_buffer[j]);
							if (j%16==0) printf("\n");
						}
						return 0;
					}					
				}
				rd_sum+=rd;
			}
			else { printf("Disconnected\n"); return 0; }
		}				
		//printf("wr:%d rd:%d\n",wr_sum,rd_sum);			
	}	
	clock_stop();
	printf("TEST1: End\n");
	ok=1;
	for (i=0;i<bytes;i++) {
		if (wr_buffer[i]!=rd_buffer[i]) ok=0;
	}			
	if(ok) 
		printf("TEST1: Successfull\n");
	else
		printf("TEST1: ERROR: received data doesnt match transmitted data\n");
	printf("TEST1: Time: %f sec\n",clock_get_sec());	
	return ok;
}	

#define	TEST2_BYTES		1000000
int test2(void)
{
	int wr_sum,wr;
	int i,ok;
	for (i=0;i<TEST2_BYTES;i++) {
		wr_buffer[i]=(unsigned char) i;
		rd_buffer[i]=0;	
	}	
	wr_sum=0;
	printf("\nTEST2: Transmit %d Bytes netsilicon->ARM7\n", TEST2_BYTES);
	printf("TEST2: Start\n");
	clock_start();
	FD_ZERO(&writers);
	while (wr_sum<TEST2_BYTES) {
		FD_SET(spi_fd, &writers);
		select(spi_fd+1,NULL,&writers,NULL,NULL);
		wr=write(spi_fd,&wr_buffer[wr_sum],TEST2_BYTES-wr_sum);
		if (wr>0) wr_sum+=wr;
		else { printf("Disconnected\n"); return 0; }
	}	
	clock_stop();
	printf("TEST2: End\n");
	ok=1;
	//TODO calculate and receive checksum
	if(ok) 
		printf("TEST2: Successfull\n");
	else
		printf("TEST2: ERROR");
	printf("TEST2: Time: %f sec\n",clock_get_sec());		
	return ok;
	
}	
#define	TEST3_BYTES		1000000
int test3(void)
{
	int rd_sum,rd;
	int i,ok;
	for (i=0;i<TEST3_BYTES;i++) {
		wr_buffer[i]=(unsigned char) i;
		rd_buffer[i]=0;	
	}	
	rd_sum=0;
	printf("\nTEST3: Transmit %d Bytes ARM7->netsilicon\n", TEST3_BYTES);
	printf("TEST3: Start\n");
	clock_start();
	FD_ZERO(&readers);
	while (rd_sum<TEST3_BYTES) {
		FD_SET(spi_fd, &readers);
		select(spi_fd+1,&readers,NULL,NULL,NULL);
		rd=read(spi_fd,&rd_buffer[rd_sum],TEST3_BYTES-rd_sum);
		if (rd>0) rd_sum+=rd;
		else { printf("Disconnected\n"); return 0; }
	}	
	clock_stop();
	printf("TEST3: End\n");
	ok=1;
	//TODO calculate and receive checksum
	if(ok) 
		printf("TEST3: Successfull\n");
	else
		printf("TEST3: ERROR");
	printf("TEST3: Time: %f sec\n",clock_get_sec());		
	return ok;
	
}	
#define	TEST4_CYCLES		20
int test4(void)
{
	int cycles;
	int wr_sum,wr,wr_size;
	int i,ok;
	for (i=0;i<PACKET_DATA_LEN*(DMA_BUFFER_NUM*2+1)*DMA_BUFFER_NUM;i++) {
		wr_buffer[i]=(unsigned char) i;
	}	

	cycles=0;
	printf("\nTEST4: Pause and initiate transfer with different size of data. netsilicon->ARM7\n");
	printf("TEST4: Start\n");
	clock_start();
	for(cycles=0;cycles<TEST4_CYCLES;cycles++) {
		for (wr_size=PACKET_DATA_LEN;wr_size<=PACKET_DATA_LEN*DMA_BUFFER_NUM*2;wr_size+=PACKET_DATA_LEN) {
			wr_sum=0;
			while (wr_sum<wr_size) {
				wr=write(spi_fd,&wr_buffer[wr_sum],wr_size-wr_sum);
				if (wr>0) wr_sum+=wr;
			}
			ndelay(1000000); //1ms				
		}
	}	
	clock_stop();
	printf("TEST4: End\n");
	ok=1;
	//TODO calculate and receive checksum
	if(ok) 
		printf("TEST4: Successfull\n");
	else
		printf("TEST4: ERROR");
	printf("TEST4: Time: not available\n");		
	return ok;
	
}

#define	TEST5_CYCLES	20
int test5(void)
{
	int rd_sum,rd,rd_size;
	int i,ok;
	for (i=0;i<TEST5_CYCLES*(PACKET_DATA_LEN*(DMA_BUFFER_NUM*2+1)*DMA_BUFFER_NUM);i++) {
		rd_buffer[i]=0;	
	}	
	rd_sum=0;
	rd_size=TEST5_CYCLES*(PACKET_DATA_LEN*(DMA_BUFFER_NUM*2+1)*DMA_BUFFER_NUM);
	printf("\nTEST5: Pause and initiate transfer with different size of data. ARM7->netsilicon\n");
	printf("TEST5: Start\n");
	clock_start();
	while (rd_sum<rd_size) {
			rd=read(spi_fd,&rd_buffer[rd_sum],rd_size-rd_sum);
			if (rd>0) rd_sum+=rd;
	}	
	clock_stop();
	printf("TEST5: End\n");
	ok=1;
	//TODO calculate and receive checksum
	if(ok) 
		printf("TEST5: Successfull\n");
	else
		printf("TEST5: ERROR");
	printf("TEST5: Time: %f sec\n",clock_get_sec());		
	return ok;
	
}	

int test6(void)
{
	int wr_sum,wr,wr_size;
	int rd_sum,rd,rd_size;
	int i,ok;
	for (i=0;i<PACKET_DATA_LEN*(DMA_BUFFER_NUM*2*10+1)*DMA_BUFFER_NUM*10;i++) {
		wr_buffer[i]=(unsigned char) i;
		rd_buffer[i]=0;	
	}	

	rd_sum=0;
	rd_size=PACKET_DATA_LEN*(DMA_BUFFER_NUM*2*10+1)*DMA_BUFFER_NUM*10;
	printf("\nTEST6: Pause and initiate transfer with different size of data. netsilicon<->ARM7\n");
	printf("TEST6: Start\n");
	clock_start();
	for (wr_size=PACKET_DATA_LEN;wr_size<=PACKET_DATA_LEN*DMA_BUFFER_NUM*2*10;wr_size+=PACKET_DATA_LEN) {
			wr_sum=0;
			while (wr_sum<wr_size) {
				wr=write(spi_fd,&wr_buffer[wr_sum],wr_size-wr_sum);
				if (wr>0) wr_sum+=wr;
				rd=read(spi_fd,&rd_buffer[rd_sum],rd_size-rd_sum);
				if (rd>0) rd_sum+=rd;
			}
			ndelay(1000000); //1ms				

	}	
	//read remaining data
	while (rd_sum<rd_size) {
			rd=read(spi_fd,&rd_buffer[rd_sum],rd_size-rd_sum);
			if (rd>0) rd_sum+=rd;
	}	
	clock_stop();
	printf("TEST6: End\n");
	ok=1;
	//TODO calculate and receive checksum
	if(ok) 
		printf("TEST6: Successfull\n");
	else
		printf("TEST6: ERROR");
	printf("TEST6: Time: not available\n");		
	return ok;
	
}


void ndelay(long ns)
{
	struct timespec req;
	req.tv_sec=0;
	req.tv_nsec = ns;
	nanosleep(&req,NULL);
}	
	
void print_stats(void)
{
	FILE *fp_stat;
	int c;
	printf("Stats:\n");
	if ((fp_stat=fopen("/proc/driver/ns_spi/stats","r")) ==	NULL) {
		printf("Error opening /proc/driver/ns_spi/stats\n");
		return;
	}
	while ((c=getc(fp_stat))!=EOF) putc(c,stdout);
}	

struct tms clock_tms1, clock_tms2;
clock_t clock_times1, clock_times2;
void clock_start(void)
{
	clock_times1=times(&clock_tms1);
}
void clock_stop(void)
{
	clock_times2=times(&clock_tms2);
}
float clock_get_sec(void)
{
	return (float)(clock_times2-clock_times1)/sysconf(_SC_CLK_TCK);
}
