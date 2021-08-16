#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DATASIZE 64
#define MAXSTACKHEIGHT 2
typedef struct FenNode{
	char data[DATASIZE];
	struct FenNode *nextNode;
} FenNode;

FenNode *bottomStackPointer = NULL;
FenNode *topStackPointer = NULL;

int push(char[]);
int pop(char[]);
void printStack(void);

extern int stackheight;
stackheight = 0;

void printStack(void){
	printf("***FenStack***\n");
	printf("**************\n");
	int numberoffNodes = 0;
	FenNode *TempPointer = bottomStackPointer;
	while(TempPointer != NULL){
		numberoffNodes++;
		printf("%s\n",TempPointer->data);
		TempPointer = TempPointer->nextNode;
	};
	printf("**************\n");
	printf("Counted %d Nodes\n",numberoffNodes);
	printf("***FenStack***\n");
	return;
}

int pop(char data[]){
	if(bottomStackPointer == NULL) return 0;
	strcpy(data,bottomStackPointer->data);
	FenNode *TempPointer = bottomStackPointer;
	bottomStackPointer = bottomStackPointer->nextNode;
	free(TempPointer);
	if(bottomStackPointer == NULL) topStackPointer = NULL;
        stackheight--;
	return 1;
}

int push(char data[]){

        if (stackheight > MAXSTACKHEIGHT)
        {
            strcpy(data,bottomStackPointer->data);
	    FenNode *TempPointer = bottomStackPointer;
	    bottomStackPointer = bottomStackPointer->nextNode;
	    free(TempPointer);
	    if(bottomStackPointer == NULL) topStackPointer = NULL;
            stackheight--;
        }

	FenNode *TempPointer = malloc(sizeof(FenNode));
	if(TempPointer == NULL) return 0;

	strcpy(TempPointer->data,data);
	TempPointer->nextNode = NULL;
	if(bottomStackPointer == NULL) TempPointer = bottomStackPointer = TempPointer;
	else topStackPointer->nextNode = TempPointer;
	topStackPointer = TempPointer;
        stackheight++;
	return 1;
}