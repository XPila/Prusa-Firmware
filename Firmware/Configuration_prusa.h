
#define PRN_VARIANT_UNDEFINED         0
#define PRN_VARIANT_MK34_MK3          1
#define PRN_VARIANT_MK34_trapez_MK3   2
#define PRN_VARIANT_MK34_X            3
#define PRN_VARIANT_MK34_trapez_X     4

#ifndef PRN_VARIANT
#define PRN_VARIANT PRN_VARIANT_UNDEFINED
#endif

#if (PRN_VARIANT == PRN_VARIANT_UNDEFINED)
#error "undefined PRN_VARIANT"
#elif (PRN_VARIANT == PRN_VARIANT_MK34_MK3)
#include "Configuration_prusa_MK34_MK3.h"
#elif (PRN_VARIANT == PRN_VARIANT_MK34_trapez_MK3)
#include "Configuration_prusa_MK34_trapez_MK3.h"
#elif (PRN_VARIANT == PRN_VARIANT_MK34_trapez_X)
#include "Configuration_prusa_MK34_trapez_X.h"
#else
#error "unknown PRN_VARIANT"
#endif
