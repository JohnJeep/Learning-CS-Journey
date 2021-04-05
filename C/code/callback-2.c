/*
 * @Author: JohnJeep
 * @Date: 2021-04-05 19:22:06
 * @LastEditTime: 2021-04-05 22:37:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */

/*
 * This is a simple C program to demonstrate the usage of callbacks
 * The callback function is in the same file as the calling code.
 * The callback function can later be put into external library like
 * e.g. a shared object to increase flexibility.
 *
 */

#include <stdio.h>
#include <string.h>

typedef struct _MyMsg {
    int id;
    char name[32];
} MyMsg;

void myfunc(MyMsg *msg)
{
    if (strlen(msg->name) > 0 ) {
        printf("id = %d \name = %s \n",msg->id, msg->name);
    }
    else {
        printf("id = %d \name = No name\n",msg->id);
    }
}

/*
 * Prototype declaration
 */
void (*callback)(MyMsg*);

int main(void)
{
    MyMsg msg1;
    msg1.id = 100;
    strcpy(msg1.name, "This is a test\n");

    /*
     * Assign the address of the function "myfunc" to the function
     * pointer "callback" (may be also written as "callback = &myfunc;")
     */
    callback = myfunc;

    /*
     * Call the function (may be also written as "(*callback)(&msg1);")
     */
    callback(&msg1);

    return 0;
}