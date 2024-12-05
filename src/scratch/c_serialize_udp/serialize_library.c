#include "serialize_library.h"

/**
 * @function serialize
 */
size_t serialize(const struct Data* data, unsigned char** buf)
{
   if (!data) return 0;

   const size_t fn_size   = data->first_name ? strlen(data->first_name) : 0;
   const size_t ln_size   = data->last_name ? strlen(data->last_name) : 0;
   const size_t data_size = sizeof(data->age) + sizeof(size_t) + fn_size + sizeof(size_t) + ln_size;
   
   *buf = calloc(1, data_size);

   if (*buf)
   {
      size_t offset = 0;
  
      // age
      unsigned int tmp = htonl(data->age);
      memcpy(*buf + offset, &tmp, sizeof(tmp));                offset += sizeof(tmp);

      // length of first name and the first name itself
      size_t tmp_l = htonl(fn_size);
      memcpy(*buf + offset, &tmp_l, sizeof(tmp_l));            offset += sizeof(tmp_l);
      memcpy(*buf + offset, data->first_name, fn_size);        offset += fn_size;

      // length of last name and the last name itself
      tmp_l = htonl(ln_size);
      memcpy(*buf + offset, &tmp_l, sizeof(tmp_l));            offset += sizeof(tmp_l);
      memcpy(*buf + offset, data->last_name, ln_size);         
   }

   return data_size;
}

/**
 * @function deserialize
 */
struct Data* deserialize(const unsigned char* buf, const size_t bufSize)
{  
   static const size_t MIN_BUF_SIZE = 12;  // min size of buffer includes age, zero-length first/last names

   struct Data* data = 0;

   if (buf && bufSize >= MIN_BUF_SIZE)
   {  
      data = calloc(1, sizeof(struct Data));
      if (data)
      {
         unsigned int tmp_i = 0;
         size_t tmp = 0;
         size_t offset = 0;

         // get the age
          memcpy(&tmp_i, buf + offset, sizeof(tmp_i));
         tmp_i = ntohl(tmp_i);
         memcpy(&data->age, &tmp_i, sizeof(data->age));
         offset  += sizeof(data->age);

         // get the first name's length
         memcpy(&tmp, buf + offset, sizeof(tmp));
         tmp = ntohl(tmp);
         offset  += sizeof(tmp);

         if (tmp > 0)
         {
            // get the first name
            data->first_name = calloc(1, tmp + 1);
            memcpy(data->first_name, buf + offset, tmp);    // missing error checks!!!
            offset  += tmp;
         }

         // get the last name's length
         memcpy(&tmp, buf + offset, sizeof(tmp));
         tmp = ntohl(tmp);
         offset  += sizeof(tmp);

         if (tmp > 0)
         { 
            // get the last name
            data->last_name = calloc(1, tmp + 1);
            memcpy(data->last_name, buf + offset, tmp);    // missing error checks!!!
         }
      }
   }

   return data;
}
