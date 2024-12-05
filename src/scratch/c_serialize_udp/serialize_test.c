#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <assert.h>
#include <stdio.h>

#include "serialize_library.h"

int main()
{
   struct Data data1;

   data1.age        = 20;
   data1.first_name = strdup("Chuck");
   data1.last_name  = strdup("Roast");

   unsigned char* buf     = 0;
   size_t         bufSize = serialize(&data1, &buf);

   struct Data*   data2   = deserialize(buf, bufSize, 0);

   // Verify serialization/deserialization worked as expected.
   assert(data2 != 0);
   assert(data1.age == data2->age);
   assert(strcmp(data1.first_name, data2->first_name) == 0);
   assert(strcmp(data1.last_name,  data2->last_name)  == 0);

   free(data1.first_name);
   free(data1.last_name);
   free(data2->first_name);
   free(data2->last_name);
   free(data2);

   return 0;
}
