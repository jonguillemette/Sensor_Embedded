#include <stdio.h>
#include <stdint.h>
#include <string.h>

//#define POLY 0x8408 
#define POLY 0x1021 
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/

uint16_t getCrcCCITT16(uint8_t * buffer, uint32_t size);
void genInitPKT(uint16_t crc);
void genManifestJSON(uint16_t crc);

int main(void)
{
	FILE * fid;
	char fname[] = "build/main.bin";
	uint32_t fsize;
	uint8_t data[256*1024];
	uint16_t crc;
	
	fid = fopen(fname, "r");
	if(fid == NULL)
	{
		printf("-> ERROR: unable to open file [%s]\n", fname);
		return 0;
	}
	fseek(fid, 0, SEEK_END);
	fsize = ftell(fid);
	fseek(fid, 0, SEEK_SET);
	fread(data, fsize, 1, fid);
	crc = getCrcCCITT16(data, fsize);
	
    printf("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww\n");
    printf("w CRC CCITT 16 calculation app\n");
    printf("w input:\t[%s]\n", fname);
    printf("w size: \t[%d]\n", fsize);
    printf("w CRC:  \t[0x%04X]\n", crc);
    printf("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww\n");
    fclose(fid);
    
    genInitPKT(crc);
    genManifestJSON(crc);
    
    
    return 0; 
}

//  <---   2B   ---> <---	 2B    --->
// | Device Type	| Device revision  |
// | Application version			   |
// | SoftDev. List N| SoftDev. Ver N   |
// | SoftDev. VerN-1|  CRC CCITT 16	   |

//0100 FEFF – any Soft Device (for development purposes only)
//0100 5A00 – compatible with the SD 7.1.0
//0200 4F00 5A00 – compatible with both SD 7.0.0 and SD 7.1.0
//0100 6400 – compatible with SD 8.0 only

void genInitPKT(uint16_t crc)
{
	uint8_t pkt[14], k;
	FILE * fid;
	
	fid = fopen("build/main.dat", "w+");
	if(fid == NULL)
	{
		printf("-> ERROR: unable to create file [main.dat]\n");
	}
	
	for(k=0;k<8;k++)
		pkt[k] = 0xFF;
	
	pkt[8] = 0x01;
	pkt[9] = 0x00;
	pkt[10] = 0x64;
	pkt[11] = 0x00;
	pkt[12] = crc&0xFF;
	pkt[13] = (crc>>8);
	
	fwrite(pkt, 14, 1, fid);
	fclose(fid);
}

void genManifestJSON(uint16_t crc)
{
	uint8_t data[2048], tstr[128];
	uint16_t k, len;
	FILE * fid;
	
	for(k=0;k<2048;k++)
		data[k] = 0x00;
		
	fid = fopen("build/manifest.json", "w+");
	if(fid == NULL)
	{
		printf("-> ERROR: unable to create file [manifest.json]\n");
	}
	//{
	strcat(data, "{\n");
		//"manifest": {
	strcat(data, "\t\"manifest\": {\n");
			//"application": {
	strcat(data, "\t\t\"application\": {\n");
				//"bin_file": "main.bin",
	strcat(data, "\t\t\t\"bin_file\": \"main.bin\",\n");
				//"dat_file": "main.dat",
	strcat(data, "\t\t\t\"dat_file\": \"main.dat\",\n");
				//"init_packet_data": {
	strcat(data, "\t\t\t\"init_packet_data\": {\n");
					//"application_version": 4294967295,
	strcat(data, "\t\t\t\t\"application_version\": 4294967295,\n");
					//"device_revision": 65535,
	strcat(data, "\t\t\t\t\"device_revision\": 65535,\n");
					//"device_type": 65535,
	strcat(data, "\t\t\t\t\"device_type\": 65535,\n");
					//"firmware_crc16": 19263,
	sprintf(tstr, "\t\t\t\t\"firmware_crc16\": %d,\n",crc);
	strcat(data, tstr);					
					//"softdevice_req": [
	strcat(data, "\t\t\t\t\"softdevice_req\": [\n");
						//100
	strcat(data, "\t\t\t\t\t100\n");
					//]
	strcat(data, "\t\t\t\t]\n");
				//}
	strcat(data, "\t\t\t}\n");
			//}
	strcat(data, "\t\t}\n");
		//}
	strcat(data, "\t}\n");
	//}
	strcat(data, "}\n");
	
	len = strlen(data);
	
	
	fwrite(data, len, 1, fid);
	fclose(fid);
}

//{
    //"manifest": {
        //"application": {
            //"bin_file": "main.bin",
            //"dat_file": "main.dat",
            //"init_packet_data": {
                //"application_version": 4294967295,
                //"device_revision": 65535,
                //"device_type": 65535,
                //"firmware_crc16": 19263,
                //"softdevice_req": [
                    //100
                //]
            //}
        //}
    //}
//}

uint16_t getCrcCCITT16(uint8_t * p_data, uint32_t size)
{
    uint32_t i;
    uint16_t crc = 0xffff;

    for (i = 0; i < size; i++)
    {
        crc  = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }

    return crc;
}


//uint16_t getCrcCCITT16(uint8_t * data_p, uint32_t size)
//{
	//uint32_t i;
    //uint16_t data;
    //uint16_t crc = 0xffff;

	//if (size == 0)
		//return (~crc);

	//do
	//{
		//for (i=0, data=(unsigned int)0xff & *data_p++;
			 //i < 8; 
			 //i++, data >>= 1)
		//{
			  //if ((crc & 0x0001) ^ (data & 0x0001))
					//crc = (crc >> 1) ^ POLY;
			  //else  crc >>= 1;
		//}
	//} while (--size);

	//crc = ~crc;
	//data = crc;
	//crc = (crc << 8) | (data >> 8 & 0xff);

	//return (crc);
//}


