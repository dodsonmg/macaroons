// Copyright 2020 Michael Dodson

#include <string>
#include <iostream>

#include "macaroon.hpp"

/* macaroons */
#include "macaroons.h"

Macaroon::Macaroon(const std::string location, const std::string key, const std::string identifier)
{
    int result = create_macaroon(location, key, identifier);

    if (result != 0) { abort(); }
}

Macaroon::Macaroon(const std::string M_serialised)
{
    int result = deserialise(M_serialised);

    if (result != 0) { abort(); }
}

Macaroon::Macaroon()
{
    M_ = NULL;
}

std::string
Macaroon::serialise()
{
    enum macaroon_returncode err;
    size_t buf_sz = 0;
    unsigned char* buf = NULL;

    /* DEBUG */
    // print_macaroon(M_);
    /* END DEBUG */

    // If M is NULL
    if(!M_)
    {
        // std::cout << "The macaroon is NULL" << std::endl;
        return "";
    }

    buf_sz = macaroon_serialize_size_hint(M_, MACAROON_V1);
    buf = (unsigned char*)malloc(buf_sz);

    macaroon_serialize(M_, MACAROON_V1, buf, buf_sz, &err);

    std::string serialised( (const char *)buf );

    // std::cout << serialised << std::endl;

    return serialised;
}

int
Macaroon::add_first_party_caveat(const std::string predicate)
{
    enum macaroon_returncode err;
    const unsigned char* ppredicate = (const unsigned char*)predicate.c_str();
    size_t ppredicate_sz = predicate.size();

    M_ = macaroon_add_first_party_caveat(M_, ppredicate, ppredicate_sz, &err);

    if(!M_)
    {
        std::cout << "Caveat addition failed" << std::endl;
        print_macaroon_error(err);
        return -1;
    }

    return 0;
}

int
Macaroon::add_third_party_caveat(void)
{
    return -1;
}

int
Macaroon::deserialise(std::string M_serialised)
{
    const unsigned char* data = (const unsigned char*)M_serialised.c_str();
    size_t data_sz = M_serialised.size();
    enum macaroon_returncode err;

    M_ = macaroon_deserialize(data, data_sz, &err);

    if(!M_)
    {
        std::cout << "Macaroon creation failed" << std::endl;
        print_macaroon_error(err);
        return -1;
    }

    /* DEBUG */
    // print_macaroon(M_);
    /* END DEBUG */

    return 0;
}

int
Macaroon::create_macaroon(const std::string location, const std::string key, const std::string identifier)
{
    enum macaroon_returncode err;

    const unsigned char* plocation = (const unsigned char*)location.c_str();
    const unsigned char* pkey = (const unsigned char*)key.c_str();
    const unsigned char* pidentifier = (const unsigned char*)identifier.c_str();

    size_t location_sz = location.size();
    size_t key_sz = key.size();
    size_t identifier_sz = identifier.size();

    M_ = macaroon_create(plocation, location_sz, 
                        pkey, key_sz, 
                        pidentifier, identifier_sz, &err);

    /* DEBUG */
    // print_macaroon(M);
    /* END DEBUG */

    // if macaroon creation fails, M will be NULL and an error code (hopefully) returned
    if(!M_)
    {
        std::cout << "Macaroon creation failed" << std::endl;
        print_macaroon_error(err);        
        return -1;
    }

    return 0;
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

struct macaroon*
Macaroon::get_macaroon_raw()
{
    return M_;
}

void
Macaroon::print_macaroon_error(enum macaroon_returncode err)
{
  std::cout << "Error (" << err << "): ";
  switch(err) {
      case MACAROON_SUCCESS : std::cout << "MACAROON_SUCCESS" << std::endl; break;
      case MACAROON_OUT_OF_MEMORY : std::cout << "MACAROON_OUT_OF_MEMORY" << std::endl; break;
      case MACAROON_HASH_FAILED : std::cout << "MACAROON_HASH_FAILED" << std::endl; break;
      case MACAROON_INVALID : std::cout << "MACAROON_INVALID" << std::endl; break;
      case MACAROON_TOO_MANY_CAVEATS : std::cout << "MACAROON_TOO_MANY_CAVEATS" << std::endl; break;
      case MACAROON_CYCLE : std::cout << "MACAROON_CYCLE" << std::endl; break;
      case MACAROON_BUF_TOO_SMALL : std::cout << "MACAROON_BUF_TOO_SMALL" << std::endl; break;
      case MACAROON_NOT_AUTHORIZED : std::cout << "MACAROON_NOT_AUTHORIZED" << std::endl; break;
      case MACAROON_NO_JSON_SUPPORT : std::cout << "MACAROON_NO_JSON_SUPPORT" << std::endl; break;
      case MACAROON_UNSUPPORTED_FORMAT : std::cout << "MACAROON_UNSUPPORTED_FORMAT" << std::endl; break;
  }
}

void
Macaroon::print_macaroon()
{
    enum macaroon_returncode err;
    size_t data_sz = 0;
    char* data = NULL;
    data_sz = macaroon_inspect_size_hint(M_);
    data = (char*)malloc(data_sz);

    macaroon_inspect(M_, data, data_sz, &err);
    std::cout << data << std::endl;
}