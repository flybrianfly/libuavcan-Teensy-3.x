/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: C:\Users\CyberPalin\Desktop\uavcan\protocol\1.GetNodeInfo.uavcan
 */

#ifndef UAVCAN_PROTOCOL_GETNODEINFO_HPP_INCLUDED
#define UAVCAN_PROTOCOL_GETNODEINFO_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/protocol/HardwareVersion.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/SoftwareVersion.hpp>

/******************************* Source text **********************************
#
# Full node info request.
# Note that all fields of the response section are byte-aligned.
#

---

#
# Current node status
#
NodeStatus status

#
# Version information shall not be changed while the node is running.
#
SoftwareVersion software_version
HardwareVersion hardware_version

#
# Human readable non-empty ASCII node name.
# Node name shall not be changed while the node is running.
# Empty string is not a valid node name.
# Allowed characters are: a-z (lowercase ASCII letters) 0-9 (decimal digits) . (dot) - (dash) _ (underscore).
# Node name is a reversed internet domain name (like Java packages), e.g. "com.manufacturer.project.product".
#
uint8[<=80] name
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.GetNodeInfo
---
uavcan.protocol.NodeStatus status
uavcan.protocol.SoftwareVersion software_version
uavcan.protocol.HardwareVersion hardware_version
saturated uint8[<=80] name
******************************************************************************/

#undef status
#undef software_version
#undef hardware_version
#undef name

namespace uavcan
{
namespace protocol
{

struct UAVCAN_EXPORT GetNodeInfo_
{
    template <int _tmpl>
    struct Request_
    {
        typedef const Request_<_tmpl>& ParameterType;
        typedef Request_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
        };

        enum
        {
            MinBitLen
        };

        enum
        {
            MaxBitLen
        };

        // Constants

        // Fields

        Request_()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<0 == MaxBitLen>::check();
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

    };

    template <int _tmpl>
    struct Response_
    {
        typedef const Response_<_tmpl>& ParameterType;
        typedef Response_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
            typedef ::uavcan::protocol::NodeStatus status;
            typedef ::uavcan::protocol::SoftwareVersion software_version;
            typedef ::uavcan::protocol::HardwareVersion hardware_version;
            typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 80 > name;
        };

        enum
        {
            MinBitLen
                = FieldTypes::status::MinBitLen
                + FieldTypes::software_version::MinBitLen
                + FieldTypes::hardware_version::MinBitLen
                + FieldTypes::name::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::status::MaxBitLen
                + FieldTypes::software_version::MaxBitLen
                + FieldTypes::hardware_version::MaxBitLen
                + FieldTypes::name::MaxBitLen
        };

        // Constants

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::status >::Type status;
        typename ::uavcan::StorageType< typename FieldTypes::software_version >::Type software_version;
        typename ::uavcan::StorageType< typename FieldTypes::hardware_version >::Type hardware_version;
        typename ::uavcan::StorageType< typename FieldTypes::name >::Type name;

        Response_()
            : status()
            , software_version()
            , hardware_version()
            , name()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<3015 == MaxBitLen>::check();
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

    };

    typedef Request_<0> Request;
    typedef Response_<0> Response;

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindService };
    enum { DefaultDataTypeID = 1 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.GetNodeInfo";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

private:
    GetNodeInfo_(); // Don't create objects of this type. Use Request/Response instead.
};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool GetNodeInfo_::Request_<_tmpl>::operator==(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
bool GetNodeInfo_::Request_<_tmpl>::isClose(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
int GetNodeInfo_::Request_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

template <int _tmpl>
int GetNodeInfo_::Request_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

template <int _tmpl>
bool GetNodeInfo_::Response_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        status == rhs.status &&
        software_version == rhs.software_version &&
        hardware_version == rhs.hardware_version &&
        name == rhs.name;
}

template <int _tmpl>
bool GetNodeInfo_::Response_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(status, rhs.status) &&
        ::uavcan::areClose(software_version, rhs.software_version) &&
        ::uavcan::areClose(hardware_version, rhs.hardware_version) &&
        ::uavcan::areClose(name, rhs.name);
}

template <int _tmpl>
int GetNodeInfo_::Response_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::status::encode(self.status, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::software_version::encode(self.software_version, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::hardware_version::encode(self.hardware_version, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::name::encode(self.name, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int GetNodeInfo_::Response_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::status::decode(self.status, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::software_version::decode(self.software_version, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::hardware_version::decode(self.hardware_version, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::name::decode(self.name, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
inline ::uavcan::DataTypeSignature GetNodeInfo_::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xA80DC8995053E685ULL);

    Response::FieldTypes::status::extendDataTypeSignature(signature);
    Response::FieldTypes::software_version::extendDataTypeSignature(signature);
    Response::FieldTypes::hardware_version::extendDataTypeSignature(signature);
    Response::FieldTypes::name::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef GetNodeInfo_ GetNodeInfo;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::GetNodeInfo > _uavcan_gdtr_registrator_GetNodeInfo;

}

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::GetNodeInfo::Request >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::GetNodeInfo::Request::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::GetNodeInfo::Request >::stream(Stream& s, ::uavcan::protocol::GetNodeInfo::Request::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
}

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::GetNodeInfo::Response::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response >::stream(Stream& s, ::uavcan::protocol::GetNodeInfo::Response::ParameterType obj, const int level)
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
    s << "status: ";
    YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response::FieldTypes::status >::stream(s, obj.status, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "software_version: ";
    YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response::FieldTypes::software_version >::stream(s, obj.software_version, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "hardware_version: ";
    YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response::FieldTypes::hardware_version >::stream(s, obj.hardware_version, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "name: ";
    YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response::FieldTypes::name >::stream(s, obj.name, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::GetNodeInfo::Request::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::GetNodeInfo::Request >::stream(s, obj, 0);
    return s;
}

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::GetNodeInfo::Response::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::GetNodeInfo::Response >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_GETNODEINFO_HPP_INCLUDED