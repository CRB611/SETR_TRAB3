#include "../unity/unity.h"
#include"../modules/uart.h"




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

    unsigned char test1[5]="M100", test2[15]="QUERTY1234abcd";

    int res = calcChecksum(&test1[0],4);

    TEST_ASSERT_EQUAL_INT(222,res);

    res = calcChecksum(&test2[0],14);

    TEST_ASSERT_EQUAL_INT(62,res);   

}
