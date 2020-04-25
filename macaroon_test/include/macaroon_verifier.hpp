// Copyright 2020 Michael Dodson

#ifndef MACAROON_VERIFIER_HPP_
#define MACAROON_VERIFIER_HPP_

#include <string>

/* copied from composition node header files */
#include "visibility.h"

/* macaroons */
#include "macaroon.hpp"
#include "macaroons.h"

class MacaroonVerifier
{
  public:
    MINIMAL_COMPOSITION_PUBLIC MacaroonVerifier(std::string key);

    int satisfy_exact(const std::string predicate);
    int satisfy_general(const std::string predicate);
    int verify(Macaroon M);  // eventually this needs to take in a tree of macaroons for 3rd party verifiers

  private:
    void print_verifier_error(enum macaroon_returncode err);

    struct macaroon_verifier* V_;
    std::string key_;
};

#endif  // MACAROON_VERIFIER_HPP_
