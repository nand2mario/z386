#include "dhry.h"

char *
strcpy (char *dst, const char *src)
{
  char *ret = dst;

  while ((*dst++ = *src++) != 0)
    ;

  return ret;
}

int
strcmp (const char *lhs, const char *rhs)
{
  while (*lhs != 0 && *lhs == *rhs)
  {
    lhs++;
    rhs++;
  }

  return (int)(unsigned char)*lhs - (int)(unsigned char)*rhs;
}

void *
memcpy (void *dst, const void *src, unsigned int len)
{
  char *d = (char *)dst;
  const char *s = (const char *)src;

  while (len-- != 0)
    *d++ = *s++;

  return dst;
}

void *
memset (void *dst, int value, unsigned int len)
{
  unsigned char *d = (unsigned char *)dst;

  while (len-- != 0)
    *d++ = (unsigned char)value;

  return dst;
}
