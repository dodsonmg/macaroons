// Copyright 2020 Michael Dodson

#ifndef MACAROON_HPP_
#define MACAROON_HPP_

#include <string>

/* copied from composition node header files */
#include "minimal_composition/visibility.h"

/* macaroons */
#include "macaroons.h"

class Macaroon
{
public:
  MINIMAL_COMPOSITION_PUBLIC Macaroon(const std::string location, const std::string key, const std::string identifier);
  MINIMAL_COMPOSITION_PUBLIC Macaroon(const std::string M_serialised);
  /* TODO: Create a copy contstructor */
  /* TODO: Create a destructor */

  std::string serialise();
  int add_first_party_caveat(const std::string predicate);
  int add_third_party_caveat(void);  // not implemented
  bool initialised();
  void print_macaroon();
  struct macaroon* get_macaroon_raw();

private:
  int create_macaroon(const std::string location, const std::string key, const std::string identifier);
  int deserialise(std::string M_serialised);
  void print_macaroon_error(enum macaroon_returncode err);

  struct macaroon* M_;
  std::string M_serialised_;
};

#endif  // MACAROON_HPP_
