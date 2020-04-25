// Copyright 2020 Michael Dodson

#include <string>
#include <iostream>

#include "minimal_composition/macaroon_verifier.hpp"
#include "minimal_composition/macaroon.hpp"

/* macaroons */
#include "macaroons.h"

MacaroonVerifier::MacaroonVerifier(std::string key)
{
    V_ = macaroon_verifier_create();
    key_ = key;
}

int
MacaroonVerifier::satisfy_exact(const std::string predicate)
{
    /*
    adds a caveat to the verifier that must be satisfied exactly

    returns 0 if added successfully, -1 otherwise
    */

    int result;
    const unsigned char* ppredicate = (const unsigned char*)predicate.c_str();
    size_t ppredicate_sz = predicate.size();
    enum macaroon_returncode err;

    result = macaroon_verifier_satisfy_exact(V_, ppredicate, ppredicate_sz, &err);

    if(result != 0)
    {
        std::cout << "Error in MacaroonVerifier::satisfy_exact:" << std::endl;
        print_verifier_error(err);
    }

    return result;
}

int
MacaroonVerifier::satisfy_general(const std::string predicate)
{
    /*
    adds a caveat to the verifier that is satisfied generally

    returns 0 if added successfully, -1 otherwise

    not implemented yet, so just returns -1 (i.e., failure)
    */
   std::cout << "MacaroonVerifier::satisfy_general() is not implemented yet." << std::endl;
   std::cout << "Attempting to set predicate " << predicate << std::endl;
   
   return -1;
}

int
MacaroonVerifier::verify(Macaroon M){
    /*
    verifies a macaroon M against the private verifier V_

    returns 0 if added successfully, -1 otherwise

    TODO:  eventually this needs to take in a tree of macaroons for 3rd party verifiers 
    */    

    int result;
    enum macaroon_returncode err;
    const unsigned char* pkey = (const unsigned char*)key_.c_str();  // uses private variable
    size_t pkey_sz = key_.size();

    // eventually these should be implemented
    struct macaroon** MS = NULL;
    size_t MS_sz = 0;

    result = macaroon_verify(V_, M.get_macaroon_raw(), pkey, pkey_sz, MS, MS_sz, &err);

    if(result != 0)
    {
        std::cout << "Error in MacaroonVerifier::verify:" << std::endl;
        print_verifier_error(err);
    }

    return result;
}

void
MacaroonVerifier::print_verifier_error(enum macaroon_returncode err)
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