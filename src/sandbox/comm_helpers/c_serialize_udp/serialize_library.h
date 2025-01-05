#ifndef _SERIALIZE_LIBRARY_
#define _SERIALIZE_LIBRARY_

#include <string.h>


struct Data
{
   unsigned int age; // 4 bytes
   char*        first_name;
   char*        last_name;
};

size_t serialize(const struct Data* data, unsigned char** buf);
struct Data* deserialize(const unsigned char* buf, const size_t bufSize, size_t start_offset);

#endif // _SERIALIZE_LIBRARY_
