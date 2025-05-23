#include "../unity/unity.h"
#include"../modules/uart.h"
#include"../src/main.c"


void setup(void){
    return;
}
 
void teardown(void){
    return;
}

void test_num2char(void){
	
	int a=23, b=200, c=5; 
	
	unsigned char result[3];

    num2char(&result[0],a);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("023",&result,3);

    num2char(&result[0],b);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("200",&result,3);
    
    num2char(&result[0],c);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("005",&result,3);
}

void test_char2num(void){

	unsigned char number[3]={'1','2','3'};

	unsigned int numberint=char2num(number,3);

	TEST_ASSERT_EQUAL_INT(123,numberint);
}

void test_char2float(void){

	unsigned char c1[5]="30.4",c2[5]="09.2", c3[5]="53.0";

	float numberint=char2float(c1);

	TEST_ASSERT_EQUAL_FLOAT(30.4,numberint);

	numberint=char2float(c2);

	TEST_ASSERT_EQUAL_FLOAT(09.2,numberint);

	numberint=char2float(c3);

	TEST_ASSERT_EQUAL_FLOAT(53.0,numberint);
}

void test_checksum(void){

    char test1[5]="M100", test2[15]="QUERTY1234abcd";

    int res = calcChecksum(&test1[0],4);

    TEST_ASSERT_EQUAL_INT(222,res);

    res = calcChecksum(&test2[0],14);

    TEST_ASSERT_EQUAL_INT(62,res);   

}

void test_commands(void){
    int err;

    /* M COMMAND*/
    rx_msg="#M100222!"
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(OK,err);
    
    uint8_t res=rtdb_get_maxtemp();
    TEST_ASSERT_EQUAL_UINT8(100,res);
    
    /* S COMMAND*/
    rx_msg="#S10.203.400.5156!";
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(OK,err);
    
    rtdb_pid help=rtdb_get_pid();
    TEST_ASSERT_EQUAL_FLOAT(10.2f,help.Kp);
    TEST_ASSERT_EQUAL_FLOAT(3.4f,help.Td);
    TEST_ASSERT_EQUAL_FLOAT(0.5f,help.Ti);

    /* R COMMAND*/
    rx_msg="#R082!";
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(OK,err);
    
    res=rtdb_get_maxtemp(); 
    help=rtdb_get_pid();
    TEST_ASSERT_EQUAL_UINT8(90,res);
    TEST_ASSERT_EQUAL_FLOAT(3.0f,help.Kp);
    TEST_ASSERT_EQUAL_FLOAT(30.0f,help.Td);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,help.Ti);
 
    /* C COMMAND*/
    rx_msg="#C067!";
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(OK,err);

    /* Buffer with more that only the message*/
    rx_msg="setr#M100222!20";
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(OK,err);
    res=rtdb_get_maxtemp();
    TEST_ASSERT_EQUAL_UINT8(100,res);
    
}

void test_command_errors(void){
    int err;

    /* WRONG CHECKSUM*/
    rx_msg="#M100232!"
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(CHECKSUM_ERROR,err);

    /* NO SOF*/
    rx_msg="M100232!"
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(SOF_ERROR,err);

    /* NO EOF*/
    rx_msg="#R082"
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(EOF_ERROR,err);
    
    /* NON EXISTENT COMMAND*/
    rx_msg="#A065!"
    err=uart_process();
    TEST_ASSERT_EQUAL_INT(COMMAND_ERROR,err);
}

