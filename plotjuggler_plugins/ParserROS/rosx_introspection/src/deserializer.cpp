#include "rosx_introspection/deserializer.hpp"
#include <fastcdr/Cdr.h>

/* make sure to remain compatible with previous version of fastCdr */
#if ((FASTCDR_VERSION_MAJOR < 2))
#define get_buffer_pointer() getBufferPointer()
#define get_current_position() getCurrentPosition()
#define CdrVersion Cdr
#endif
namespace RosMsgParser
{

Variant ROS_Deserializer::deserialize(BuiltinType type)
{
  switch (type)
  {
    case BOOL:
      return deserialize<bool>();
    case CHAR:
      return deserialize<char>();
    case BYTE:
    case UINT8:
      return deserialize<uint8_t>();
    case UINT16:
      return deserialize<uint16_t>();
    case UINT32:
      return deserialize<uint32_t>();
    case UINT64:
      return deserialize<uint64_t>();

    case INT8:
      return deserialize<int8_t>();
    case INT16:
      return deserialize<int16_t>();
    case INT32:
      return deserialize<int32_t>();
    case INT64:
      return deserialize<int64_t>();

    case FLOAT32:
      return deserialize<float>();
    case FLOAT64:
      return deserialize<double>();

    case DURATION:
    case TIME: {
      RosMsgParser::Time tmp;
      tmp.sec = deserialize<uint32_t>();
      tmp.nsec = deserialize<uint32_t>();
      return tmp;
    }

    default:
      std::runtime_error("ROS_Deserializer: type not recognized");
  }

  return {};
}

void ROS_Deserializer::deserializeString(std::string& dst)
{
  uint32_t string_size = deserialize<uint32_t>();

  if (string_size > _bytes_left)
  {
    throw std::runtime_error("Buffer overrun in ROS_Deserializer::deserializeString");
  }

  if (string_size == 0)
  {
    dst = {};
    return;
  }

  const char* buffer_ptr = reinterpret_cast<const char*>(_ptr);
  dst.assign(buffer_ptr, string_size);

  _ptr += string_size;
  _bytes_left -= string_size;
}

uint32_t ROS_Deserializer::deserializeUInt32()
{
  return deserialize<uint32_t>();
}

Span<const uint8_t> ROS_Deserializer::deserializeByteSequence()
{
  uint32_t vect_size = deserialize<uint32_t>();
  if (vect_size > _bytes_left)
  {
    throw std::runtime_error("Buffer overrun in "
                             "ROS_Deserializer::deserializeByteSequence");
  }
  if (vect_size == 0)
  {
    return {};
  }
  Span<const uint8_t> out(_ptr, vect_size);
  jump(vect_size);
  return out;
}

const uint8_t* ROS_Deserializer::getCurrentPtr() const
{
  return _ptr;
}

void ROS_Deserializer::jump(size_t bytes)
{
  if (bytes > _bytes_left)
  {
    throw std::runtime_error("Buffer overrun");
  }
  _ptr += bytes;
  _bytes_left -= bytes;
}

void ROS_Deserializer::reset()
{
  _ptr = _buffer.data();
  _bytes_left = _buffer.size();
}

// ----------------------------------------------

template <typename T>
static T Deserialize(eprosima::fastcdr::Cdr& cdr)
{
  T tmp;
  cdr.deserialize(tmp);
  return tmp;
}

Variant FastCDR_Deserializer::deserialize(BuiltinType type)
{
  switch (type)
  {
    case BOOL:
      return Deserialize<bool>(*_cdr);
    case CHAR:
      return Deserialize<char>(*_cdr);
    case BYTE:
    case UINT8:
      return Deserialize<uint8_t>(*_cdr);
    case UINT16:
      return Deserialize<uint16_t>(*_cdr);
    case UINT32:
      return Deserialize<uint32_t>(*_cdr);
    case UINT64:
      return Deserialize<uint64_t>(*_cdr);

    case INT8:
      return Deserialize<int8_t>(*_cdr);
    case INT16:
      return Deserialize<int16_t>(*_cdr);
    case INT32:
      return Deserialize<int32_t>(*_cdr);
    case INT64:
      return Deserialize<int64_t>(*_cdr);

    case FLOAT32:
      return Deserialize<float>(*_cdr);
    case FLOAT64:
      return Deserialize<double>(*_cdr);

    case DURATION:
    case TIME: {
      RosMsgParser::Time tmp;
      tmp.sec = Deserialize<uint32_t>(*_cdr);
      tmp.nsec = Deserialize<uint32_t>(*_cdr);
      return tmp;
    }

    default:
      throw std::runtime_error("FastCDR_Deserializer: type not recognized");
  }

  return {};
}

void FastCDR_Deserializer::deserializeString(std::string& dst)
{
  _cdr->deserialize(dst);
}

uint32_t FastCDR_Deserializer::deserializeUInt32()
{
  return Deserialize<uint32_t>(*_cdr);
}

Span<const uint8_t> FastCDR_Deserializer::deserializeByteSequence()
{
  //  thread_local std::vector<uint8_t> tmp;
  //  _cdr->deserialize(tmp);
  //  return {tmp.data(), tmp.size()};

  uint32_t seqLength = 0;
  _cdr->deserialize(seqLength);

  // dirty trick to change the internal state of cdr
  auto* ptr = _cdr->get_current_position();

  uint8_t dummy;
  _cdr->deserialize(dummy);

  _cdr->jump(seqLength - 1);
  return { reinterpret_cast<const uint8_t*>(ptr), seqLength };
}

const uint8_t* FastCDR_Deserializer::getCurrentPtr() const
{
  return reinterpret_cast<const uint8_t*>(_cdr->get_buffer_pointer());
}

void FastCDR_Deserializer::jump(size_t bytes)
{
  _cdr->jump(bytes);
}

void FastCDR_Deserializer::reset()
{
  using namespace eprosima::fastcdr;

  char* buffer_ptr = reinterpret_cast<char*>(const_cast<uint8_t*>(_buffer.data()));

  _cdr_buffer = std::make_shared<FastBuffer>(buffer_ptr, _buffer.size());
  _cdr = std::make_shared<Cdr>(*_cdr_buffer, Cdr::DEFAULT_ENDIAN, CdrVersion::DDS_CDR);
  _cdr->read_encapsulation();
}

}  // namespace RosMsgParser
