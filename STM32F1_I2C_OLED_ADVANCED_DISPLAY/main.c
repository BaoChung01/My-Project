#include "systick_time.h"
#include "i2c_drive.h"
#include "oled_drive.h"

unsigned char oled_buffer[8][132];

int main(void)
{
	int i,j;
	systick_init();
	oled_init_64(2);
		
	oled_blank(2);
	
	
	// Start of the Nokia 5110 Code (to be converted)

	//ImgType tho_pic = {.h = tho_rows, .w = tho_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	//tho_pic.image[0] = tho;
	
/*
	ImgType walkmen = {.h = walkmen_4_rows, .w = walkmen_4_cols, .x_pos = 0, .y_pos =0};
	walkmen.image[0] = walkmen_0;
	walkmen.image[1] = walkmen_1;
	walkmen.image[2] = walkmen_2;
	walkmen.image[3] = walkmen_3;
	walkmen.image[4] = walkmen_4;

	ImgType anh1 = {.h = fairy_rows, .w = fairy_cols, .x_pos = 35, .y_pos =0};
	anh1.image[0] = fairy;
	ImgType anh2 = {.h = logo_rows, .w = logo_cols, .x_pos = 35, .y_pos =0};
	anh2.image[0] = logo;
	*/
	ImgType error_pic = {.h = error_rows, .w = error_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	error_pic.image[0] = error;

	ImgType happy_pic = {.h = happy_rows, .w = happy_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	happy_pic.image[0] = happy;
	
	ImgType sad_pic = {.h = sad_rows, .w = sad_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	sad_pic.image[0] = sad;
	
	ImgType a_pic = {.h = a_rows, .w = a_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	a_pic.image[0] = a;
	
	ImgType b_pic = {.h = b_rows, .w = b_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	b_pic.image[0] = b;
	
	ImgType c_pic = {.h = c_rows, .w = c_cols, .x_pos = 25, .y_pos =0, .bit_y_pos = 0};
	c_pic.image[0] = c;
	
	clear_buffer(oled_buffer);
	update_buffer(a_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(3000);
	
	clear_buffer(oled_buffer);
	update_buffer(b_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(3000);
	
	clear_buffer(oled_buffer);
	update_buffer(c_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(3000);
	
	clear_buffer(oled_buffer);
	update_buffer(error_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(3000);
	
	clear_buffer(oled_buffer);
	update_buffer(happy_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(3000);
	
	clear_buffer(oled_buffer);
	update_buffer(sad_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(3000);
/*
	clear_buffer(oled_buffer);
	update_buffer(anh2, 0, oled_buffer);
	print_buffer(2,oled_buffer);
	DelayMs(5000);
	*/
	
		/*for(i=0;i<50;i++)
	{
	tho_pic.x_pos = 20*i;
	tho_pic.bit_y_pos =60; // dich sang phai
	//tho_pic.x_pos = 3*i;
	//tho_pic.bit_y_pos =2*;  // dich sang cheo
	clear_buffer(oled_buffer);
	Digital_Output(PC,13);
		DelayMs(500);
		toggle_GP(PC,13);
	update_buffer_bit(tho_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
		DelayMs(20);
	}
	*/
	
	/*for(i=50;i>0;i--)
	{
	tho_pic.x_pos = 20*i;
	tho_pic.bit_y_pos =60;
	clear_buffer(oled_buffer);
	Digital_Output(PC,13);
	DelayMs(500);
	toggle_GP(PC,13);
	update_buffer_bit(tho_pic, 0, oled_buffer);
	print_buffer(2,oled_buffer);
		DelayMs(20);
	}*/
	oled_blank(2);
	oled_msg(2,3,25,"EMBEDDED SV");
	DelayMs(1000);
	
	
	while(1)
	{
		
		//Digital_Output(PC,13);
		//DelayMs(500);
		//toggle_GP(PC,13);
		/*for (i=0;i<83;i++)
		{
			walkmen.x_pos = i;
		if(i== 40)
		{
			clear_buffer(oled_buffer);
			update_buffer(walkmen, 4, oled_buffer);
			update_str_buffer(5, 15,"** Just Do It **",oled_buffer);
			print_buffer(2,oled_buffer);
			DelayMs(1000);
		}else{
		for(j=0;j<4;j++)
		{
			clear_buffer(oled_buffer);
			update_buffer(walkmen, j, oled_buffer);
			print_buffer(2,oled_buffer);
			DelayMs(100);
		}
	}
		}*/
	}
	
}

