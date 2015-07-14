#include<stdio.h>


public int main(int argv char* argc[]) {

	ParseClient client = parseInitialize("LLXKP3xsmyHpEsZiYo6b8i9kHhsHDKyrlkW5lNrP", "D8XJySU9yqmTTLQkMDLEebVfKmLjp1ApNtWuFyxN");
	 
	parseSendRequest(client, "POST", "/1/classes/TestObject", "{\"foo\":\"bar\"}", NULL);
}
