///////////////////////////////////////
/// 640x480 version!
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc mandelbrot_video_2.c -o mandel -mfpu=neon -ftree-vectorize -O2
/// -- no optimization yields 9100 mS execution time
/// -- opt -O1 yields 4800 mS execution time
/// -- opt -O2 yields 3500 mS execution time
/// -- opt -O3 yields 3400 mS execution time
/// -- adding 'f' to all floating literals (e.g. 1.3f) and opt -O2 yields 2640 mS execution time
///gcc mandelbrot_video_vector.c -o mandel -mfpu=neon -ftree-vectorize 
///               -O3 -mfloat-abi=hard -ffast-math 
/// -- yields 2544 mS execution time
/// change to fixed point 
/// gcc mandelbrot_video_fix.c -o mandel -Os
/// -- yields 1615 mS execution time
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include "address_map_arm_brl4.h"
#include <pthread.h>
#include <time.h>
/* function prototypes */
void VGA_quarter (int, int, short, int);
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_disc (int, int, int, short);
void delay(int);
void VGA_rect(int, int, int, int, short);
void VGA_cello(int, int, int);
void VGA_marimba(int, int, int);
void VGA_oboe(int, int, int);

// fixed pt
typedef signed int fix28 ;
//multiply two fixed 4:28
#define multfix28(a,b) ((fix28)(((( signed long long)(a))*(( signed long long)(b)))>>28)) 
//#define multfix28(a,b) ((fix28)((( ((short)((a)>>17)) * ((short)((b)>>17)) )))) 
#define float2fix28(a) ((fix28)((a)*268435456.0f)) // 2^28
#define fix2float28(a) ((float)(a)/268435456.0f) 
#define int2fix28(a) ((a)<<28);
// the fixed point value 4
#define FOURfix28 0x40000000 
#define SIXTEENTHfix28 0x01000000
#define ONEfix28 0x10000000

#define FPGA_PIO_RESET	0x60
// the light weight bus base
void *h2p_lw_virtual_base;
// HPS_to_FPGA FIFO status address = 0
volatile unsigned int * lw_pio_ptr = NULL ;
volatile unsigned int * lw_pio_read_ptr = NULL ;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// shared memory 
key_t mem_key=0xf0;
int shared_mem_id; 
int *shared_ptr;
int shared_time;
int shared_note;
char shared_str[64];

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	char  *pixel_ptr ;\
	pixel_ptr = (char *)vga_pixel_ptr + ((y)<<10) + (x) ;\
	*(char *)pixel_ptr = (color);\
} while(0)
	
// mandelbrot iteration max
#define max_count 1000

// mandelbrot arrays
fix28 Zre, Zim, Cre, Cim, Zre_temp, Zim_temp;
fix28 Zre_sq, Zim_sq ;
fix28 x[640], y[480];
int i, j, count, total_count;

// measure time
struct timeval t1, t2;
double elapsedTime;

short note_color = 0xffff;
volatile int state = 0; // 0 welcome, 1 room


void * write_VGA(){
 char text_top_row[40] = "WELCOME TO BOLERO SURROUND SOUND!\0";
        char text_by[5] = "By:\0";
        char text_names[40] = "Owen Valdescruz and Diane Sutyak\0";
        char text_bottom_row[40] = "Cornell ece5760\0";
        char num_string[20], time_string[20] ;

        //VGA_text (34, 1, text_top_row);
        //VGA_text (34, 2, text_bottom_row);
        // clear the screen
        VGA_box (0, 0, 639, 479, 0x0);
        // clear the text
        VGA_text_clear();

        VGA_text (27, 20, text_top_row);
  VGA_text (38, 33, text_by);
  VGA_text (25, 35, text_names);
        VGA_text (32, 40, text_bottom_row);

	while(state == 0) {
		VGA_quarter(80, 270, rand() & 0xff, 10);
		VGA_quarter(520, 270, rand() & 0xff, 10);
		delay(1000);
	}
  int current_state = state;
  while(1){
  
	 // DRAW ROOM OUTLINE
	// clear the screen
	VGA_box (0, 0, 639, 479, 0xff);
	// clear the text
	VGA_text_clear();
	// write walls
	VGA_rect(160, 100, 480, 380, 0x0);
	VGA_line(160, 100, 0, 0, 0x0);
	VGA_line(160, 380, 0, 479, 0x0);
	VGA_line(480, 100, 639, 0, 0x0);
	VGA_line(480, 380, 639, 479, 0x0);
	VGA_rect(2, 2, 638, 478, 0x0);
 
  if(state==2) {
    VGA_cello(70,400, 8);
    VGA_marimba(320, 350, 5);
    VGA_oboe(550, 370, 4);
  }
  else if(state==4) {
    VGA_cello(550, 390, 8);
    VGA_marimba(75,385, 5);
    VGA_oboe(320, 350, 4);
	}
  else if(state==6) {
    VGA_cello(320, 350, 8);
    VGA_marimba(550, 370, 5);
    VGA_oboe(70, 400, 4);
	}

   while(current_state == state);
   current_state = state;
	//VGA_cello(70,400, 8);
  //VGA_marimba(550, 370, 5);
  //VGA_oboe(320, 350, 4);
  }
}

void * read_terminal(){
	unsigned int input;
	
		int num, pio_read;
		int junk; 
     num = 1;
   *(lw_pio_ptr)  = num ;
		// input a number
    printf("Press Enter to begin\n");
    getchar();
    num = 0;
    *(lw_pio_ptr)  = num ;
    state = 2;
    while(1) 
	{
    printf("Press 2 to turn left, 3 to turn right\n");
		junk = scanf("%d", &num);
    if(state==2) {
      if(num==2) {state = 4; num=4;}
      else if(num==3) {state = 6; num=6;}
    }
    else if(state==4) {
      if(num==2) {state = 6; num=6;}
      else if(num==3) {state = 2; num=2;}
		}
    else if(state==6) {
      if(num==2) {state = 2; num=2;}
      else if(num==3) {state = 4; num=4;}
		}
		// send to PIOs
		*(lw_pio_ptr)  = num ;
   
	}
	
}





	
int main(void)
{
	//int x1, y1, x2, y2;

	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 

	// === shared memory =======================
	// with video process
	shared_mem_id = shmget(mem_key, 100, IPC_CREAT | 0666);
 	//shared_mem_id = shmget(mem_key, 100, 0666);
	shared_ptr = shmat(shared_mem_id, NULL, 0);

  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
    
   // Get the addresses that map to the two parallel ports on the light-weight bus
	lw_pio_ptr = (unsigned int *)(h2p_lw_virtual_base + FPGA_PIO_RESET);

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, 			FPGA_ONCHIP_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================


	pthread_t thread_terminal, thread_VGA;
	
	pthread_create(&thread_terminal, NULL, read_terminal, NULL);
	pthread_create(&thread_VGA, NULL, write_VGA, NULL);
	
	pthread_join(thread_terminal, NULL);
	pthread_join(thread_VGA, NULL);


	/* create a message to be displayed on the VGA 
          and LCD displays */
/*	char text_top_row[40] = "WELCOME TO BOLERO SURROUND SOUND!\0";
	char text_by[5] = "By:\0";
	char text_names[40] = "Owen Valdescruz and Diane Sutyak\0";
	char text_bottom_row[40] = "Cornell ece5760\0";
	char num_string[20], time_string[20] ;

	//VGA_text (34, 1, text_top_row);
	//VGA_text (34, 2, text_bottom_row);
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x0);
	// clear the text
	VGA_text_clear();
 
 	VGA_text (27, 20, text_top_row);
  VGA_text (38, 33, text_by);
  VGA_text (25, 35, text_names);
	VGA_text (32, 40, text_bottom_row);
  

VGA_quarter(80, 270, 0xffff, 10);
  VGA_quarter(520, 270, 0xffff, 10); */
/*
	// x increment
	for (i=0; i<640; i++) {
		x[i] = float2fix28(-2.0f + 3.0f * (float)i/640.0f) ;
		//printf("%f\n", x[i]);
	}
	
	// y increment
	for (j=0; j<480; j++) {
		y[j] = float2fix28(-1.0f + 2.0f * (float)j/480.0f) ;
		//printf("%f\n", y[j]);
	}
	
	// start timer
    gettimeofday(&t1, NULL);
	
	// zero global counter
	total_count = 0 ;
	
	fix28 center = float2fix28(-0.25f);
	fix28 radius2 = float2fix28(0.25f);
	fix28 q;
	
	//while(1) 
	//{
		
		for (i=0; i<640; i++) {
			for (j=0; j<480; j+=1) {
				Zre = Zre_sq = 0 ;
				Zim = Zim_sq = 0 ;
				Cre = x[i];
				Cim = y[j];
				//count = 0 ;
				// detect secondary bulb
				 if ((multfix28(Cre+ONEfix28,Cre+ONEfix28)+multfix28(Cim,Cim))<SIXTEENTHfix28) {
					 count=max_count;
				 }
				//detect big circle
				 else if ((multfix28(Cre-center,Cre-center)+multfix28(Cim,Cim))<radius2) {
					 count=max_count;
				 }
				 else count = 0;
				
				// iterate to find convergence, or not
				while (count++ < max_count){
					//update the complex iterator
					Zim = (multfix28(Zre,Zim)<<1) + Cim ;
					Zre = Zre_sq - Zim_sq + Cre ;
					Zre_sq = multfix28(Zre, Zre) ;
					Zim_sq = multfix28(Zim, Zim);
					// check non-convergence
					if ((Zre_sq + Zim_sq) >= FOURfix28) break;
				}
				total_count += count ;
				if (count>=max_count) VGA_PIXEL(i,j,0x01);
				else if (count>=(max_count>>1)) VGA_PIXEL(i,j,0xe0); // e0
				else if (count>=(max_count>>2)) VGA_PIXEL(i,j,0xe0); //e0
				else if (count>=(max_count>>3)) VGA_PIXEL(i,j,0xc0); //e0
				else if (count>=(max_count>>4)) VGA_PIXEL(i,j,0xc0); //c0
				else if (count>=(max_count>>5)) VGA_PIXEL(i,j,0xa0); //a0
				else if (count>=(max_count>>6)) VGA_PIXEL(i,j,0x80); //80
				else if (count>=(max_count>>7)) VGA_PIXEL(i,j,0x24); //40
				else VGA_PIXEL(i,j,0x24); // dark gray
			}
		}
		VGA_text (10, 1, text_top_row);
	    VGA_text (10, 2, text_bottom_row);
		
		// stop timer
		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
		elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
		sprintf(num_string, "# = %d     ", total_count);
		sprintf(time_string, "T = %6.0f mSec  ", elapsedTime);
		VGA_text (10, 3, num_string);
		VGA_text (10, 4, time_string);
		*/
	//} // end while(1)
} // end main


void delay(int milliseconds)
{
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}

void VGA_oboe(int x, int y, int size) {
	short white = 0xff;
  short black = 0x0;
	VGA_box(x, y, x+4*size, y-1*size, black);
  VGA_box(x-2*size, y-size, x+6*size, y-2*size, black);
  VGA_box(x-3*size, y-2*size, x+7*size, y-4*size, black);
  VGA_box(x-size, y-4*size, x+5*size, y-5*size, black);
  VGA_box(x, y-5*size, x+4*size, y-34*size, black);
  VGA_box(x+size, y-34*size, x+3*size, y-38*size, black);
  
  VGA_box(x+size, y-6*size, x+2*size, y-7*size, white);
  VGA_box(x+size, y-7*size, x+4*size, y-8*size, white);
  VGA_box(x, y-9*size, x+size, y-10*size, white);
  VGA_box(x, y-11*size, x+size, y-12*size, white);
  VGA_box(x, y-13*size, x+size, y-14*size, white);
  VGA_box(x+2*size, y-16*size, x+4*size, y-17*size, white);
  VGA_box(x+2*size, y-17*size, x+3*size, y-20*size, white);
  VGA_box(x, y-21*size, x+size, y-22*size, white);
  VGA_box(x, y-23*size, x+size, y-24*size, white);
  VGA_box(x, y-25*size, x+size, y-26*size, white);
  VGA_box(x+3*size, y-28*size, x+4*size, y-29*size, white);
  VGA_box(x+3*size, y-30*size, x+4*size, y-31*size, white);
  
  
 }

void VGA_marimba(int x, int y, int size) {
	short maroon = 0xc0;
  short black = 0x0;
	VGA_box(x-12*size, y, x-13*size, y-7*size, black);
  VGA_box(x-10*size, y-9*size, x+12*size, y-13*size, black);
  VGA_box(x+14*size, y, x+15*size, y-7*size, black);
  VGA_box(x+13*size, y-6*size, x+14*size, y-8*size, black);
  VGA_box(x+12*size, y-7*size, x+13*size, y-9*size, black);
  VGA_box(x+11*size, y-8*size, x+12*size, y-10*size, black);
  //VGA_box(x+10*size, y-9*size, x+11*size, y-11*size, black);
  VGA_box(x-11*size, y-6*size, x-12*size, y-8*size, black);
  VGA_box(x-10*size, y-7*size, x-11*size, y-9*size, black);
  VGA_box(x-9*size, y-8*size, x-10*size, y-10*size, black);
  
  VGA_box(x-7*size, y-8*size, x+10*size, y-9*size, black);
  VGA_box(x-5*size, y-7*size, x+10*size, y-8*size, black);
  VGA_box(x-2*size, y-6*size, x+10*size, y-7*size, black);
  VGA_box(x+1*size, y-5*size, x+10*size, y-6*size, black);
  VGA_box(x+4*size, y-4*size, x+10*size, y-5*size, black);
  VGA_box(x+7*size, y-3*size, x+10*size, y-4*size, black);
  
  VGA_box(x-9*size, y-10*size, x-7*size, y-12*size, maroon);
  VGA_box(x-6*size, y-10*size, x-4*size, y-12*size, maroon);
  VGA_box(x-3*size, y-10*size, x-1*size, y-12*size, maroon);
  VGA_box(x, y-10*size, x+2*size, y-12*size, maroon);
  VGA_box(x+3*size, y-10*size, x+5*size, y-12*size, maroon);
  VGA_box(x+6*size, y-10*size, x+8*size, y-12*size, maroon);
  VGA_box(x+9*size, y-10*size, x+11*size, y-12*size, maroon);
  
  
}

void VGA_cello(int x, int y, int size) {
	short brown = 0x44;
  short white = 0xff;
  short black = 0x0;
	VGA_box(x, y, x+size, y-size, brown);
	VGA_box(x-2*size, y-size, x+3*size , y-2*size, brown);
	VGA_box(x-3*size, y-2*size, x+4*size, y-3*size, brown);
	VGA_box(x-5*size, y-3*size, x+6*size, y-19*size, brown);
	VGA_box(x-5*size, y-4*size, x-6*size, y-10*size, brown);
	VGA_box(x-6*size, y-5*size, x-5*size, y-9*size, brown);
	VGA_box(x+6*size, y-4*size, x+7*size, y-10*size, brown);
	VGA_box(x+7*size, y-5*size, x+8*size, y-9*size, brown);
  VGA_box(x-6*size, y-5*size, x-7*size, y-9*size, brown);
  VGA_box(x-5*size, y-13*size, x-6*size, y-18*size, brown);
  VGA_box(x-6*size, y-14*size, x-7*size, y-17*size, brown);
  VGA_box(x+6*size, y-13*size, x+7*size, y-18*size, brown);
  VGA_box(x+7*size, y-14*size, x+8*size, y-17*size, brown);
  VGA_box(x-4*size, y-19*size, x+5*size, y-20*size, brown);
  VGA_box(x-3*size, y-20*size, x+4*size, y-21*size, brown);
  
  VGA_box(x-size, y-6*size, x, y-8*size, black);
  VGA_box(x+size, y-6*size, x+2*size, y-8*size, black);
  VGA_box(x-size, y-11*size, x, y-28*size, black);
  VGA_box(x+size, y-11*size, x+2*size, y-28*size, black);
  VGA_box(x, y-3*size, x+size, y-6*size, black);
  VGA_box(x, y-6*size, x+size, y-8*size, white);
  VGA_box(x, y-11*size, x+size, y-28*size, white);
  VGA_box(x-size, y-28*size, x+2*size, y-29*size, black);
  VGA_box(x, y-29*size, x+size, y-31*size, white);
  
  VGA_box(x-5*size, y-8*size, x-3*size, y-9*size, black);
  VGA_box(x-4*size, y-8*size, x-3*size, y-13*size, black);
  VGA_box(x-3*size, y-12*size, x-2*size, y-13*size, black);
  VGA_box(x+4*size, y-8*size, x+6*size, y-9*size, black);
  VGA_box(x+4*size, y-8*size, x+5*size, y-13*size, black);
  VGA_box(x+3*size, y-12*size, x+5*size, y-13*size, black);
  
}


void VGA_quarter(int x, int y, short pixel_color, int size) {
  VGA_box(x+size, y, x+3*size, y-size, pixel_color);
  VGA_box(x, y-size, x+4*size, y-size*3, pixel_color);
  VGA_box(x+size, y-3*size, x+4*size, y-size*4, pixel_color);
  VGA_box(x+3*size, y-4*size, x+4*size, y-size*11, pixel_color);
  VGA_box(x+4*size, y-9*size, x+6*size, y-size*11, pixel_color);
  VGA_box(x+5*size, y-8*size, x+7*size, y-size*10, pixel_color);
  VGA_box(x+6*size, y-7*size, x+8*size, y-size*9, pixel_color);
  VGA_box(x+6*size, y-6*size, x+7*size, y-size*7, pixel_color);
  VGA_box(x+5*size, y-5*size, x+6*size, y-size*6, pixel_color);
}


/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a outline rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	// left edge
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		*(char *)pixel_ptr = pixel_color;		
	}
		
	// right edge
	col = x2;
	for (row = y1; row <= y2; row++){
		//640x480
		pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		*(char *)pixel_ptr = pixel_color;		
	}
	
	// top edge
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		*(char *)pixel_ptr = pixel_color;		
	}
	
	// bottom edge
	row = y2;
	for (col = x1; col <= x2; ++col){
		//640x480
		pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		*(char *)pixel_ptr = pixel_color;		
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			*(char *)pixel_ptr = pixel_color;		
		}
}

/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				*(char *)pixel_ptr = pixel_color;
			}
					
		}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		*(char *)pixel_ptr = c;	
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}

