/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: C:\Users\CyberPalin\Desktop\uavcan\protocol\debug\16383.LogMessage.uavcan
 */

#ifndef UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/protocol/debug/LogLevel.hpp>

/******************************* Source text **********************************
#
# Generic log message.
# All items are byte aligned.
#

LogLevel level
uint8[<=31] source
uint8[<=90] text
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.debug.LogMessage
uavcan.protocol.debug.LogLevel level
saturated uint8[<=31] source
saturated uint8[<=90] text
******************************************************************************/

#undef level
#undef source
#undef text

namespace uavcan
{
namespace protocol
{
namespace debug
{

template <int _tmpl>
struct UAVCAN_EXPORT LogMessage_
{
    typedef const LogMessage_<_tmpl>& ParameterType;
    typedef LogMessage_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::protocol::debug::LogLevel level;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 31 > source;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 90 > text;
    };

    enum
    {
        MinBitLen
            = FieldTypes::level::MinBitLen
            + FieldTypes::source::MinBitLen
            + FieldTypes::text::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::level::MaxBitLen
            + FieldTypes::source::MaxBitLen
            + FieldTypes::text::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::level >::Type level;
    typename ::uavcan::StorageType< typename FieldTypes::source >::Type source;
    typename ::uavcan::StorageType< typename FieldTypes::text >::Type text;

    LogMessage_()
        : level()
        , source()
        , text()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<983 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 16383 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.debug.LogMessage";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool LogMessage_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        level == rhs.level &&
        source == rhs.source &&
        text == rhs.text;
}

template <int _tmpl>
bool LogMessage_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(level, rhs.level) &&
        ::uavcan::areClose(source, rhs.source) &&
        ::uavcan::areClose(text, rhs.text);
}

template <int _tmpl>
int LogMessage_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::level::encode(self.level, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::source::encode(self.source, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::text::encode(self.text, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int LogMessage_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::level::decode(self.level, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::source::decode(self.source, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::text::decode(self.text, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature LogMessage_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xE9862B78D38762BAULL);

    FieldTypes::level::extendDataTypeSignature(signature);
    FieldTypes::source::extendDataTypeSignature(signature);
    FieldTypes::text::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef LogMessage_<0> LogMessage;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::debug::LogMessage > _uavcan_gdtr_registrator_LogMessage;

}

} // Namespace debug
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::debug::LogMessage >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::debug::LogMessage::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::debug::LogMessage >::stream(Stream& s, ::uavcan::protocol::debug::LogMessage::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "level: ";
    YamlStreamer< ::uavcan::protocol::debug::LogMessage::FieldTypes::level >::stream(s, obj.level, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "source: ";
    YamlStreamer< ::uavcan::protocol::debug::LogMessage::FieldTypes::source >::stream(s, obj.source, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "text: ";
    YamlStreamer< ::uavcan::protocol::debug::LogMessage::FieldTypes::text >::stream(s, obj.text, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace debug
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::debug::LogMessage::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::debug::LogMessage >::stream(s, obj, 0);
    return s;
}

} // Namespace debug
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_HPP_INCLUDED