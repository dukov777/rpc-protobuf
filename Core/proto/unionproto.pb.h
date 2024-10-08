/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9 */

#ifndef PB_UNIONPROTO_PB_H_INCLUDED
#define PB_UNIONPROTO_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _AccessMode {
    AccessMode_READ_ONLY = 0,
    AccessMode_READ_WRITE = 1
} AccessMode;

/* Struct definitions */
typedef struct _MsgType1 {
    int32_t value;
    bool has_access;
    AccessMode access;
} MsgType1;

typedef struct _MsgType2 {
    bool value;
    bool has_access;
    AccessMode access;
} MsgType2;

typedef struct _MsgType3 {
    int32_t value1;
    int32_t value2;
    bool has_access;
    AccessMode access;
} MsgType3;

typedef struct _UnionMessage {
    bool has_msg1;
    MsgType1 msg1;
    bool has_msg2;
    MsgType2 msg2;
    bool has_msg3;
    MsgType3 msg3;
} UnionMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _AccessMode_MIN AccessMode_READ_ONLY
#define _AccessMode_MAX AccessMode_READ_WRITE
#define _AccessMode_ARRAYSIZE ((AccessMode)(AccessMode_READ_WRITE+1))

#define MsgType1_access_ENUMTYPE AccessMode

#define MsgType2_access_ENUMTYPE AccessMode

#define MsgType3_access_ENUMTYPE AccessMode



/* Initializer values for message structs */
#define MsgType1_init_default                    {0, false, AccessMode_READ_ONLY}
#define MsgType2_init_default                    {0, false, AccessMode_READ_ONLY}
#define MsgType3_init_default                    {0, 0, false, AccessMode_READ_ONLY}
#define UnionMessage_init_default                {false, MsgType1_init_default, false, MsgType2_init_default, false, MsgType3_init_default}
#define MsgType1_init_zero                       {0, false, _AccessMode_MIN}
#define MsgType2_init_zero                       {0, false, _AccessMode_MIN}
#define MsgType3_init_zero                       {0, 0, false, _AccessMode_MIN}
#define UnionMessage_init_zero                   {false, MsgType1_init_zero, false, MsgType2_init_zero, false, MsgType3_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MsgType1_value_tag                       1
#define MsgType1_access_tag                      2
#define MsgType2_value_tag                       1
#define MsgType2_access_tag                      2
#define MsgType3_value1_tag                      1
#define MsgType3_value2_tag                      2
#define MsgType3_access_tag                      3
#define UnionMessage_msg1_tag                    1
#define UnionMessage_msg2_tag                    2
#define UnionMessage_msg3_tag                    3

/* Struct field encoding specification for nanopb */
#define MsgType1_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    value,             1) \
X(a, STATIC,   OPTIONAL, UENUM,    access,            2)
#define MsgType1_CALLBACK NULL
#define MsgType1_DEFAULT NULL

#define MsgType2_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, BOOL,     value,             1) \
X(a, STATIC,   OPTIONAL, UENUM,    access,            2)
#define MsgType2_CALLBACK NULL
#define MsgType2_DEFAULT NULL

#define MsgType3_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    value1,            1) \
X(a, STATIC,   REQUIRED, INT32,    value2,            2) \
X(a, STATIC,   OPTIONAL, UENUM,    access,            3)
#define MsgType3_CALLBACK NULL
#define MsgType3_DEFAULT NULL

#define UnionMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  msg1,              1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  msg2,              2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  msg3,              3)
#define UnionMessage_CALLBACK NULL
#define UnionMessage_DEFAULT NULL
#define UnionMessage_msg1_MSGTYPE MsgType1
#define UnionMessage_msg2_MSGTYPE MsgType2
#define UnionMessage_msg3_MSGTYPE MsgType3

extern const pb_msgdesc_t MsgType1_msg;
extern const pb_msgdesc_t MsgType2_msg;
extern const pb_msgdesc_t MsgType3_msg;
extern const pb_msgdesc_t UnionMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define MsgType1_fields &MsgType1_msg
#define MsgType2_fields &MsgType2_msg
#define MsgType3_fields &MsgType3_msg
#define UnionMessage_fields &UnionMessage_msg

/* Maximum encoded size of messages (where known) */
#define MsgType1_size                            13
#define MsgType2_size                            4
#define MsgType3_size                            24
#define UNIONPROTO_PB_H_MAX_SIZE                 UnionMessage_size
#define UnionMessage_size                        47

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
