// Copyright 2018-present, Simone Fontana
// Distributed under the GNU GPL 3.0 License (https://www.gnu.org/licenses/gpl-3.0.html)

#ifndef PSO_REGISTRATION_SWARM_HPP_
#define PSO_REGISTRATION_SWARM_HPP_

#include <omp.h>

#include <fstream>
#include <vector>

#include "pso_registration/particle.hpp"

using pso_registration::Particle;

namespace pso_registration {
class Swarm {
 public:
  Swarm() : particles_(), generator_(rd_()) {}

  void add_particle(Particle part) { particles_.push_back(part); }

  void init() {
#pragma omp parallel for num_threads(std::thread::hardware_concurrency())
    for (int i = 0; i < particles_.size(); i++) {
      particles_[i].setGlobalBest(findLBest(particles_[i].getId()));
    }

    best_ = findGBest();
  }

  Particle getBest() { return best_; }

  void evolve() {
#pragma omp parallel for num_threads(std::thread::hardware_concurrency())
    for (int j = 0; j < particles_.size(); j++) {
      particles_[j].evolve();
    }

    for (int ind = 0; ind < particles_.size(); ind++) {
      particles_[ind].setGlobalBest(findLBest(ind));
    }

    Particle gen_best = findGBest();
    if (best_.getScore() < gen_best.getScore()) {
      best_ = gen_best;
    }
  }

 private:
  std::vector<Particle> particles_;
  Particle best_;
  std::random_device rd_;
  std::mt19937 generator_;

  Particle findGBest() { return *(std::max_element(particles_.begin(), particles_.end(), Particle::cmp)); }

  Particle findLBest(int index) {
    std::vector<Particle> neighbours;
    if (index == 0) {
      neighbours.push_back(particles_.back());
    } else {
      neighbours.push_back(particles_[index - 1]);
    }

    if (index == particles_.size() - 1) {
      neighbours.push_back(particles_.front());
    } else {
      neighbours.push_back(particles_[index + 1]);
    }

    Particle lbest = *(std::max_element(neighbours.begin(), neighbours.begin(), Particle::cmp));

    return lbest;
  }
  friend std::ostream &operator<<(std::ostream &os, Swarm const &s);
};

std::ostream &operator<<(std::ostream &os, Swarm const &s) {
  for (auto &p : s.particles_) {
    os << p << std::endl;
  }
  os << "................." << std::endl;
  os << "Best: " << s.best_ << std::endl;
  os << "---------------------";
  return os;
}

}  // namespace pso_registration

#endif  // PSO_REGISTRATION_SWARM_HPP_
