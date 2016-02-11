#include "misc.h"

void getDNum(uint8_t * dnum, uint32_t num)
{/// calculate decimal digits from integer number 'num' and store them as ASCII char in dnum array
	uint8_t k;
	uint32_t step = 1000000000;
	
	for(k=0;k<10;k++)
	{// convert integer value in hex format to decimal format
		dnum[9-k] = 48;													// set char value
		while(num >= step)
		{
			dnum[9-k]++;
			num -= step;
		}
		step /= 10;
	}
}

void putDNum(uint8_t * dnum, uint16_t * m, uint8_t * r_str)
{/// add unsigned integer number digits into string r_str
	uint8_t k = 0, flag = 0;
	
	for(k=0;k<10;k++)
	{// convert integer value in hex format to decimal format
		if(flag == 0)
		{
			if((dnum[9-k] != 48) || (k == 9))					// include the 0x00000000 number
			{
				flag = 1;
				r_str[(*m)] = dnum[9-k];
				(*m)++;
			}
		}
		else
		{
			r_str[(*m)] = dnum[9-k];
			(*m)++;
		}
		
	}
}
void strcon(uint8_t * des_str, uint8_t * src_str)
{/// concatenate two strings
	uint16_t k = 0,n = 0;
	
	while(des_str[k] != '\0')
	{
		k++;
	}
	
	while(src_str[n] != '\0')
	{
		des_str[k] = src_str[n];
		n++;
		k++;
	}
	des_str[k] = '\0';
}


void getASCIIString(uint8_t * str, uint32_t  * num, uint8_t * r_str)
{/// print text and one signed integer or float number
  // the 'num' number is not modified!
	uint8_t dnum[10],flag = 0;
	uint16_t e_part;
	uint16_t n = 0, m = 0, k = 0;
	int32_t * i_ptr;
	int32_t i_num;
	uint32_t d_part, i_part;
	uint8_t sige = ' ';
	float * f_ptr;
	float f_num;
	float f_step;

	while(str[n] != '\0')
	{
		if(flag == 0)
		{// search for '%' character
			if(str[n] == '%')
			{// found it!
				flag = 1;
			}
			else
			{// this is not it! print the character
				r_str[m] = str[n];
				m++;
			}
			n++;
			
		}
		else if(flag == 1)
		{// test for number type  
			if(str[n] == 'd') 
			{// the given number is signed integer 32bit
				flag = 2;
			}
			if(str[n] == 'c')
			{
				flag = 3;
			}
			else if(str[n] == 'f')
			{// the given number is signed float 32bit
				flag = 4;
			}
			else if(str[n] == 's')
			{//  need to implement
				flag = 5;
			}
			else if(str[n] == 'b')
			{//  need to implement
				flag = 6;
			}
			else if(str[n] == 'h')
			{//  need to implement
				flag = 7;
			}
			else
			{// other conversions
			}
		}
		else if(flag == 2)
		{// add the signed integer number
			i_ptr = (int32_t *)num;
			i_num = *i_ptr;
			if(i_num < 0)	
			{											// print the sign character
				r_str[m] = '-';
				m++;
				i_num = i_num*(-1);
			}

			getDNum(dnum, (uint32_t)i_num);
			putDNum(dnum, &m, r_str);
			
			n++;
			flag = 10;
		}
		else if(flag == 3)
		{// add single character		
			r_str[m] = (char)(*num);
			m++;
			n++;
			flag = 10;
		}
		else if(flag == 4)
		{// add the float number
			f_ptr = (float *)num;										// convert pointer
			f_num = *f_ptr;												// copy value
			
			if(f_num < 0)
			{// negative float number
				r_str[m] = '-';
				m++;
				f_num = (-1)*f_num;
			}
			
			if(f_num < 1)
			{
				sige = '-';
				f_step = 0.1;
				e_part = 1;
				while(f_num < f_step)
				{
					f_step = f_step*0.1;
					e_part++;
				}	
			}
			else
			{
				f_step = 1.0;
				e_part = 0;
				while(f_num > f_step)
				{
					f_step = f_step*10;
					e_part++;
				}
				f_step = f_step/10;
				e_part--;			
			}
			
			f_num = f_num/f_step;
			i_part = (uint32_t)f_num;
			
			f_num = f_num - (float)i_part;
			d_part = (uint32_t)(1000*f_num);
			
			getDNum(dnum, i_part);
			putDNum(dnum, &m, r_str);

			getDNum(dnum, d_part);
			dnum[3] = '.';
			putDNum(dnum, &m, r_str);
			
			r_str[m] = 'e';
			m++;
			
			if(sige == '-')
			{
				r_str[m] = '-';
				m++;
			}
				
				
			getDNum(dnum, e_part);
			putDNum(dnum, &m, r_str);
			
			n++;
			flag = 10;
		}
		else if(flag == 5)
		{// add the string 
			uint8_t * t_num = (char *)num;
			while(t_num[k] != '\0')
			{
				r_str[m] = t_num[k];
				m++;
				k++;
			}
			n++;
			flag = 10;
		}		
		else if(flag == 6)
		{// add the binary data 
			uint32_t b_num = *num;
			uint8_t g = 0;
			for(g=0;g<32;g++)
			{
				if(b_num&0x80000000)
				{
					r_str[m] = '1';
				} 
				else
				{
					r_str[m] = '0';					
				}
				b_num = b_num<<1;
				m++;
			}
			n++;
			flag = 10;
		}		
		else if(flag == 7)
		{// add the hex data 
			uint32_t b_num = *num;
			uint8_t t_rez, g;
			
			r_str[m] = '0';
			r_str[m+1] = 'x';
			m+=2;
			
			for(g=0;g<8;g++)
			{
				t_rez = (b_num&0xF0000000)>>28;
				if(t_rez < 0x0A)
				{
					r_str[m] = t_rez + 0x30;
				} 
				else
				{
					r_str[m] = t_rez + 0x41 - 0x0A;					
				}
				b_num = b_num<<4;
				m++;
			}
			n++;
			flag = 10;
		}
		else
		{// add remaining characters
			r_str[m] = str[n];
			n++;
			m++;
		}
	}
	r_str[m] = '\0';
}

void catstr(uint8_t * str1, uint8_t * str2)
{// concatenate two strings
	uint16_t k=0,n=0;
	while(str1[k] != '\0')
	{
		k++;
	}
	while(str2[n] != '\0')
	{
		str1[k] = str2[n];
		k++;
		n++;
	}
	str1[k] = '\0';
} 
