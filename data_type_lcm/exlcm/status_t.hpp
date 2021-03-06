/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __exlcm_status_t_hpp__
#define __exlcm_status_t_hpp__

#include <string>

namespace exlcm
{

class status_t
{
    public:
        int32_t    robot_id;
        std::string frame_id;
        double     x_robot;
        double     y_robot;
        double     th_robot;

    public:
        inline int encode(void *buf, int offset, int maxlen) const;
        inline int getEncodedSize() const;
        inline int decode(const void *buf, int offset, int maxlen);
        inline static int64_t getHash();
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int status_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int status_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int status_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t status_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* status_t::getTypeName()
{
    return "status_t";
}

int status_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->robot_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* frame_id_cstr = (char*) this->frame_id.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &frame_id_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->x_robot, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->y_robot, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->th_robot, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int status_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->robot_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    int32_t __frame_id_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__frame_id_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__frame_id_len__ > maxlen - pos) return -1;
    this->frame_id.assign(((const char*)buf) + offset + pos, __frame_id_len__ - 1);
    pos += __frame_id_len__;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->x_robot, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->y_robot, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->th_robot, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int status_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += this->frame_id.size() + 4 + 1;
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t status_t::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0xa61740edee6e40ffLL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
