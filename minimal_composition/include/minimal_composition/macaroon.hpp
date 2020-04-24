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
  /* TODO: Create a second contstructor that takes in a serialised constructor */
  /* TODO: Create a copy contstructor */
  /* TODO: Create a destructor */

  std::string serialise();
  std::string serialise_macaroon(struct macaroon* M);
  struct macaroon* deserialise_macaroon(std::string M_serialised);
  bool initialised();

private:
  struct macaroon* create_macaroon(const std::string location, const std::string key, const std::string identifier);
  void print_macaroon_error(enum macaroon_returncode err);
  void print_macaroon(struct macaroon* M);

  struct macaroon* M_;
  std::string M_serialised_;
};

#endif  // MACAROON_HPP_
