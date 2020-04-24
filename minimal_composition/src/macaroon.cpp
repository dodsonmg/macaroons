// Copyright 2020 Michael Dodson

#include <string>
#include <iostream>

#include "minimal_composition/macaroon.hpp"

/* macaroons */
#include "macaroons.h"

Macaroon::Macaroon(const std::string location, const std::string key, const std::string identifier)
{
    M_ = create_macaroon(location, key, identifier);
}

std::string
Macaroon::serialise()
{
    enum macaroon_returncode err;
    size_t buf_sz = 0;
    unsigned char* buf = NULL;

    /* DEBUG */
    print_macaroon(M_);
    /* END DEBUG */

    // If M is NULL
    if(!M_)
    {
        std::cout << "The macaroon is NULL" << std::endl;
        return "";
    }

    buf_sz = macaroon_serialize_size_hint(M_, MACAROON_V1);
    buf = (unsigned char*)malloc(buf_sz);

    macaroon_serialize(M_, MACAROON_V1, buf, buf_sz, &err);

    std::string serialised( (const char *)buf );

    std::cout << serialised << std::endl;

    return serialised;
}

std::string
Macaroon::serialise_macaroon(struct macaroon* M)
{
    enum macaroon_returncode err;
    size_t buf_sz = 0;
    unsigned char* buf = NULL;

    /* DEBUG */
    print_macaroon(M);
    /* END DEBUG */

    // If M is NULL
    if(!M)
    {
        std::cout << "The macaroon is NULL" << std::endl;
        return "";
    }

    buf_sz = macaroon_serialize_size_hint(M, MACAROON_V1);
    buf = (unsigned char*)malloc(buf_sz);

    macaroon_serialize(M, MACAROON_V1, buf, buf_sz, &err);

    std::string serialised( (const char *)buf );

    std::cout << serialised << std::endl;

    return serialised;
}

struct macaroon*
Macaroon::deserialise_macaroon(std::string M_serialised)
{
    const unsigned char* data = (const unsigned char*)M_serialised.c_str();
    size_t data_sz = M_serialised.size();
    enum macaroon_returncode err;
    struct macaroon* M = NULL;

    M = macaroon_deserialize(data, data_sz, &err);

    if(!M) { print_macaroon_error(err); }
    else {
        /* DEBUG */
        print_macaroon(M);
        /* END DEBUG */
    }

    return M;
}

struct macaroon*
Macaroon::create_macaroon(const std::string location, const std::string key, const std::string identifier)
{
    struct macaroon* M = NULL;
    enum macaroon_returncode err;

    const unsigned char* plocation = (const unsigned char*)location.c_str();
    const unsigned char* pkey = (const unsigned char*)key.c_str();
    const unsigned char* pidentifier = (const unsigned char*)identifier.c_str();

    size_t location_sz = location.size();
    size_t key_sz = key.size();
    size_t identifier_sz = identifier.size();

    /* DEBUG */
    // std::cout << plocation << "\t" << location_sz << std::endl;
    // std::cout << pkey << "\t" << key_sz << std::endl;
    // std::cout << pidentifier << "\t" << identifier_sz << std::endl;
    /* END DEBUG */

    M = macaroon_create(plocation, location_sz, 
                        pkey, key_sz, 
                        pidentifier, identifier_sz, &err);

    /* DEBUG */
    // print_macaroon(M);
    /* END DEBUG */

    // if macaroon creation fails, M will be NULL and an error code (hopefully) returned
    if(!M)
    {
        print_macaroon_error(err);
        return NULL;
    }

    return M;
}

bool
Macaroon::initialised()
{
    if(M_)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void
Macaroon::print_macaroon_error(enum macaroon_returncode err)
{
  std::cout << "Macaroon not created" << std::endl;
  std::cout << "Error:" << err << std::endl;
  std::cout << "MACAROON_SUCCESS:" << MACAROON_SUCCESS << std::endl;
  std::cout << "MACAROON_OUT_OF_MEMORY:" << MACAROON_OUT_OF_MEMORY << std::endl;
  std::cout << "MACAROON_HASH_FAILED:" << MACAROON_HASH_FAILED << std::endl;
  std::cout << "MACAROON_INVALID:" << MACAROON_INVALID << std::endl;
  std::cout << "MACAROON_TOO_MANY_CAVEATS:" << MACAROON_TOO_MANY_CAVEATS << std::endl;
  std::cout << "MACAROON_CYCLE:" << MACAROON_CYCLE << std::endl;
  std::cout << "MACAROON_BUF_TOO_SMALL:" << MACAROON_BUF_TOO_SMALL << std::endl;
  std::cout << "MACAROON_NOT_AUTHORIZED:" << MACAROON_NOT_AUTHORIZED << std::endl;
  std::cout << "MACAROON_NO_JSON_SUPPORT:" << MACAROON_NO_JSON_SUPPORT << std::endl;
  std::cout << "MACAROON_UNSUPPORTED_FORMAT:" << MACAROON_UNSUPPORTED_FORMAT << std::endl;
}

void
Macaroon::print_macaroon(struct macaroon* M)
{
    enum macaroon_returncode err;
    size_t data_sz = 0;
    char* data = NULL;
    data_sz = macaroon_inspect_size_hint(M);
    data = (char*)malloc(data_sz);

    macaroon_inspect(M, data, data_sz, &err);
    std::cout << data << std::endl;
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)
