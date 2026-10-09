#pragma once
/*
 * Actuator friction identification for the G1 (mc_unitree2, --calib).
 *
 * When STATUS_WAITING_AIR: one joint at a time is driven through triangle
 * sweeps at several speeds with 5 s stops at both ends.
 * Record the measured torque, g(q), qdot then the gearbox friction model
 * used by mc_external_forces_observer is fitted on the SLIDING samples:
 *     tau - g = s * (Fc + mu * |g|) + Fv * qdot,   s = sign(qdot)
 * The at-rest samples are then checked with the presliding rule (friction keeps
 * the sign of the last sliding motion); the RMS of what is left is `sigma`, the
 * uncertainty the observer propagates into its wrench dead-band.
 */
#include <mc_rtc/logging.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace mc_unitree {

struct SweepSpec {
  std::string joint;  ///< motor joint name (motorJointNames)
  std::map<std::string, double>
      setup;  ///< other joints to place first {name: rad}
  double a;   ///< sweep start [rad]
  double b;   ///< sweep end [rad]
  std::vector<double> speeds{
      0.2, 0.6};      ///< traverse speeds [rad/s] (two suffice for Fc vs Fv)
  int cycles = 2;     ///< round trips per speed
  double hold = 4.0;  ///< stop at each end [s]
  std::string label;  ///< free text for the report (e.g. "elbow, arm forward")
  bool fullRange =
      false;  ///< ignore a/b: sweep the whole joint range minus the margin
};

class ActuatorCalibration {
 public:
  static constexpr int N = 41;
  using Vec = Eigen::Matrix<float, N, 1>;

  struct JointResult {
    std::string joint;
    bool identified = false;
    std::string reason;
    double Fc = 0, mu = 0, Fv = 0, sigma = 0, r2 = 0;
    double gRange = 0;
    int nPos = 0, nNeg = 0;
    double restRawMean = 0, restRawSd = 0, restCompMean = 0, restCompSd = 0;
  };

  ActuatorCalibration(double dt, const std::array<std::string, N>& names,
                      const Vec& qLimLower, const Vec& qLimUpper,
                      const Vec& qInit)
      : dt_(dt),
        names_(names),
        qLimLower_(qLimLower),
        qLimUpper_(qLimUpper),
        qRef_(qInit) {
    for (int i = 0; i < N; ++i) {
      index_[names_[static_cast<size_t>(i)]] = i;
    }
  }

  /** The default protocol: legs then arms, mirrored for the right side. Ranges
   * are clamped to the interface joint limits minus `margin` at start(). */
  static std::vector<SweepSpec> defaultTable() {
    std::vector<SweepSpec> t;
    auto mirror = [](std::map<std::string, double> m, const std::string& side) {
      std::map<std::string, double> out;
      for (auto& [k, v] : m) {
        std::string name = k;
        if (name.rfind("SIDE_", 0) == 0) {
          name = side + "_" + name.substr(5);
        }
        double val = v;
        // roll and yaw joints are mirrored between the two sides
        if (side == "right" && (name.find("_roll_") != std::string::npos ||
                                name.find("_yaw_") != std::string::npos)) {
          val = -v;
        }
        out[name] = val;
      }
      return out;
    };
    // Self-collision check with the Revo2 hands (with the URDF meshes)
    auto armsParked = [](std::map<std::string, double> m) {
      m["left_shoulder_roll_joint"] = 0.6;
      m["right_shoulder_roll_joint"] = -0.6;
      return m;
    };
    for (const std::string side : {"left", "right"}) {
      const double sg = (side == "left") ? 1.0 : -1.0;
      // ---- leg (feet free, hanging), both arms parked outward ----
      t.push_back(
          {side + "_hip_pitch_joint",
           armsParked(mirror(
               {{"SIDE_knee_joint", 0.3}, {"SIDE_hip_roll_joint", 0.0}}, side)),
           -1.0,
           0.8,
           {0.2, 0.6},
           2,
           4.0,
           side + " hip pitch, leg hanging"});
      t.push_back({side + "_hip_roll_joint",
                   armsParked(mirror({{"SIDE_knee_joint", 0.3},
                                      {"SIDE_hip_pitch_joint", 0.0}},
                                     side)),
                   -0.3 * sg,
                   0.6 * sg,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " hip roll, leg hanging"});
      t.push_back({side + "_knee_joint",
                   armsParked(mirror({{"SIDE_hip_pitch_joint", -1.57},
                                      {"SIDE_hip_roll_joint", 0.0}},
                                     side)),
                   0.1,
                   1.6,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " knee, thigh horizontal"});
      t.push_back({side + "_ankle_pitch_joint",
                   armsParked(mirror({{"SIDE_hip_pitch_joint", -1.57},
                                      {"SIDE_knee_joint", 1.57}},
                                     side)),
                   -0.6,
                   0.6,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " ankle pitch, shank horizontal",
                   true});
      // ---- arm (shoulder roll 0.5 outward keeps the hand clear of head and
      // torso) ----
      t.push_back({side + "_shoulder_pitch_joint",
                   mirror({{"SIDE_shoulder_roll_joint", 0.5},
                           {"SIDE_shoulder_yaw_joint", 0.0},
                           {"SIDE_elbow_joint", 0.3}},
                          side),
                   -1.2,
                   0.8,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " shoulder pitch"});
      t.push_back({side + "_shoulder_roll_joint",
                   mirror({{"SIDE_shoulder_pitch_joint", 0.0},
                           {"SIDE_shoulder_yaw_joint", 0.0},
                           {"SIDE_elbow_joint", 0.3}},
                          side),
                   0.2 * sg,
                   1.8 * sg,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " shoulder roll"});
      t.push_back({side + "_shoulder_yaw_joint",
                   mirror({{"SIDE_shoulder_pitch_joint", 0.0},
                           {"SIDE_shoulder_roll_joint", 1.2},
                           {"SIDE_elbow_joint", 1.2}},
                          side),
                   -1.0 * sg,
                   1.0 * sg,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " shoulder yaw, arm out"});
      t.push_back({side + "_elbow_joint",
                   mirror({{"SIDE_shoulder_pitch_joint", 0.0},
                           {"SIDE_shoulder_roll_joint", 0.5},
                           {"SIDE_shoulder_yaw_joint", 0.0}},
                          side),
                   0.1,
                   1.6,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " elbow, arm hanging"});
      t.push_back({side + "_elbow_joint",
                   mirror({{"SIDE_shoulder_pitch_joint", -1.57},
                           {"SIDE_shoulder_roll_joint", 0.5},
                           {"SIDE_shoulder_yaw_joint", 0.0}},
                          side),
                   0.1,
                   1.6,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " elbow, arm forward"});
      t.push_back({side + "_wrist_pitch_joint",
                   mirror({{"SIDE_shoulder_pitch_joint", 0.0},
                           {"SIDE_shoulder_roll_joint", 0.5},
                           {"SIDE_elbow_joint", 1.57}},
                          side),
                   -1.2,
                   1.2,
                   {0.2, 0.6},
                   2,
                   4.0,
                   side + " wrist pitch, arm straight down"});
    }
    return t;
  }

  /** Subset of the protocol: "all", "arms", "legs", "left", "right", or "quick"
   * (one arm joint, one cycle, one speed: a ~1 min pipeline test, not an
   * identification). */
  static std::vector<SweepSpec> filterTable(std::vector<SweepSpec> t,
                                            const std::string& set) {
    if (set == "all" || set.empty()) {
      return t;
    }
    if (set == "quick") {
      for (auto& s : t) {
        if (s.joint == "left_shoulder_pitch_joint") {
          s.speeds = {0.3};
          s.cycles = 1;
          s.hold = 2.0;
          return {s};
        }
      }
      return {};
    }
    std::vector<SweepSpec> out;
    for (const auto& s : t) {
      const bool arm = s.joint.find("shoulder") != std::string::npos ||
                       s.joint.find("elbow") != std::string::npos ||
                       s.joint.find("wrist") != std::string::npos;
      const bool left = s.joint.rfind("left_", 0) == 0;
      if ((set == "arms" && arm) || (set == "legs" && !arm) ||
          (set == "left" && left) || (set == "right" && !left)) {
        out.push_back(s);
      }
    }
    return out;
  }

  /** Build the timeline. Returns false if the table references unknown joints.
   */
  bool start(const std::vector<SweepSpec>& table, const Vec& qInit,
             double margin = 0.1) {
    qInit_ = qInit;
    qRef_ = qInit;
    phases_.clear();
    samples_.clear();
    results_.clear();
    Vec target = qInit;
    for (const auto& s : table) {
      auto it = index_.find(s.joint);
      if (it == index_.end()) {
        mc_rtc::log::error("[calib] unknown joint \"{}\" in the sweep table",
                           s.joint);
        return false;
      }
      const int m = it->second;
      const double lo = qLimLower_(m) + margin, hi = qLimUpper_(m) - margin;
      double a = s.fullRange ? lo : std::clamp(s.a, lo, hi),
             b = s.fullRange ? hi : std::clamp(s.b, lo, hi);
      if (s.fullRange) {
        mc_rtc::log::info(
            "[calib] {}: full range [{:.2f}, {:.2f}] (limits {:.2f}..{:.2f}, "
            "margin {:.2f})",
            s.joint, a, b, qLimLower_(m), qLimUpper_(m), margin);
      } else if (a != s.a || b != s.b) {
        mc_rtc::log::warning(
            "[calib] {}: sweep [{:.2f}, {:.2f}] clamped to [{:.2f}, {:.2f}] "
            "(limits {:.2f}..{:.2f}, margin {:.2f})",
            s.joint, s.a, s.b, a, b, qLimLower_(m), qLimUpper_(m), margin);
      }
      // 1. move to the setup pose (tested joint to a), settle
      Vec setup = target;
      for (const auto& [jn, v] : s.setup) {
        auto jt = index_.find(jn);
        if (jt == index_.end()) {
          mc_rtc::log::error("[calib] unknown setup joint \"{}\" for {}", jn,
                             s.joint);
          return false;
        }
        setup(jt->second) =
            static_cast<float>(std::clamp(v, qLimLower_(jt->second) + margin,
                                          qLimUpper_(jt->second) - margin));
      }
      setup(m) = static_cast<float>(a);
      addMove(target, setup, kSetupSpeed, -1, false, s.label + " (setup)");
      addHold(setup, 3.0, -1, false);
      target = setup;
      // 2. sweeps
      for (double v : s.speeds) {
        for (int c = 0; c < s.cycles; ++c) {
          Vec tb = target;
          tb(m) = static_cast<float>(b);
          addMove(target, tb, v, m, true, s.label);
          addHold(tb, s.hold, m, true);
          Vec ta = tb;
          ta(m) = static_cast<float>(a);
          addMove(tb, ta, v, m, true, s.label);
          addHold(ta, s.hold, m, true);
          target = ta;
        }
      }
      // 3. back to the initial pose
      addMove(target, qInit, kSetupSpeed, -1, false, s.label + " (return)");
      addHold(qInit, 2.0, -1, false);
      target = qInit;
    }
    phase_ = 0;
    tPhase_ = 0.0;
    active_ = !phases_.empty();
    totalDuration_ = 0.0;
    for (const auto& p : phases_) {
      totalDuration_ += p.duration;
    }
    mc_rtc::log::info("[calib] {} joints, {} phases, {:.0f} s planned",
                      table.size(), phases_.size(), totalDuration_);
    return active_;
  }

  bool active() const { return active_; }
  double elapsed() const { return tElapsed_; }
  double planned() const { return totalDuration_; }
  const Vec& qRef() const { return qRef_; }

  /** Advance one control step. q/dq/tau/g are motor-indexed (N). */
  void update(const Vec& q, const Vec& dq, const Vec& tau, const Vec& g) {
    if (!active_) {
      return;
    }
    const Phase& p = phases_[static_cast<size_t>(phase_)];
    const double r =
        p.duration > 0 ? std::clamp(tPhase_ / p.duration, 0.0, 1.0) : 1.0;
    qRef_ = p.from + static_cast<float>(r) * (p.to - p.from);
    if (p.record && p.joint >= 0) {
      auto& S = samples_[p.joint];
      S.push_back({tau(p.joint), g(p.joint), dq(p.joint), q(p.joint)});
    }
    if (p.joint != lastLoggedJoint_ && p.joint >= 0) {
      lastLoggedJoint_ = p.joint;
      mc_rtc::log::info("[calib] {} ({}) - {:.0f}/{:.0f} s", p.label,
                        names_[static_cast<size_t>(p.joint)], tElapsed_,
                        totalDuration_);
    }
    tPhase_ += dt_;
    tElapsed_ += dt_;
    if (tPhase_ >= p.duration) {
      tPhase_ = 0.0;
      qRef_ = p.to;
      phase_ += 1;
      if (phase_ >= static_cast<int>(phases_.size())) {
        active_ = false;
        qRef_ = qInit_;
        mc_rtc::log::info("[calib] sweep finished after {:.0f} s", tElapsed_);
      }
    }
  }

  /** Fit every recorded joint. */
  void fit(double vs = 0.02, int minSamples = 200,
           double minGravityRange = 1.0) {
    vs_ = vs;
    results_.clear();
    for (const auto& [m, S] : samples_) {
      JointResult R;
      R.joint = names_[static_cast<size_t>(m)];
      mc_rtc::log::info("[calib] fit {}: {} samples", R.joint, S.size());
      double gmin = 1e9, gmax = -1e9;
      std::vector<const Sample*> sliding;
      for (const auto& s : S) {
        gmin = std::min(gmin, s.g);
        gmax = std::max(gmax, s.g);
        if (std::abs(s.dq) >= vs) {
          sliding.push_back(&s);
          (s.dq > 0 ? R.nPos : R.nNeg) += 1;
        }
      }
      R.gRange = gmax - gmin;
      double tauMax = 0.0;
      for (const auto& s : S) {
        tauMax = std::max(tauMax, std::abs(s.tau));
      }
      if (tauMax < 1e-6) {
        R.identified = false;
        R.reason = "no torque data (loopback or sensor not reporting)";
        results_.push_back(R);
        continue;
      }
      if (R.nPos < minSamples || R.nNeg < minSamples ||
          R.gRange < minGravityRange) {
        R.identified = false;
        R.reason = fmt::format(
            "need >= {} sliding samples per direction (have {}/{}) and g range "
            ">= {:.1f} Nm (have {:.2f})",
            minSamples, R.nPos, R.nNeg, minGravityRange, R.gRange);
        results_.push_back(R);
        continue;
      }
      Eigen::MatrixXd X(static_cast<Eigen::Index>(sliding.size()), 3);
      Eigen::VectorXd y(static_cast<Eigen::Index>(sliding.size()));
      for (Eigen::Index i = 0; i < X.rows(); ++i) {
        const auto& s = *sliding[static_cast<size_t>(i)];
        const double sg = s.dq > 0 ? 1.0 : -1.0;
        X(i, 0) = sg;
        X(i, 1) = sg * std::abs(s.g);
        X(i, 2) = s.dq;
        y(i) = s.tau - s.g;
      }
      Eigen::Vector3d p = X.colPivHouseholderQr().solve(y);
      Eigen::VectorXd res = y - X * p;
      const double ss = res.squaredNorm();
      const double my = y.mean();
      const double tot = (y.array() - my).square().sum();
      R.Fc = p(0);
      R.mu = p(1);
      R.Fv = p(2);
      R.r2 = tot > 1e-12 ? 1.0 - ss / tot : 0.0;
      // at-rest check with presliding memory
      double lastDir = 0.0;
      std::vector<double> raw, comp;
      for (const auto& s : S) {
        if (std::abs(s.dq) >= vs) {
          lastDir = s.dq > 0 ? 1.0 : -1.0;
          continue;
        }
        if (lastDir == 0.0) {
          continue;
        }
        const double tauF =
            lastDir * (R.Fc + R.mu * std::abs(s.g)) + R.Fv * s.dq;
        raw.push_back(s.tau - s.g);
        comp.push_back(s.tau - s.g - tauF);
      }
      auto meanSd = [](const std::vector<double>& v, double& mean, double& sd) {
        if (v.empty()) {
          mean = sd = 0;
          return;
        }
        mean = 0;
        for (double x : v) {
          mean += x;
        }
        mean /= static_cast<double>(v.size());
        sd = 0;
        for (double x : v) {
          sd += (x - mean) * (x - mean);
        }
        sd = std::sqrt(sd / static_cast<double>(v.size()));
      };
      meanSd(raw, R.restRawMean, R.restRawSd);
      meanSd(comp, R.restCompMean, R.restCompSd);
      double rms = 0;
      for (double x : comp) {
        rms += x * x;
      }
      R.sigma = comp.empty()
                    ? std::sqrt(ss / static_cast<double>(res.size()))
                    : std::sqrt(rms / static_cast<double>(comp.size()));
      R.identified = true;
      results_.push_back(R);
    }
  }

  const std::vector<JointResult>& results() const { return results_; }
  bool identifiedAny() const {
    for (const auto& r : results_) {
      if (r.identified) {
        return true;
      }
    }
    return false;
  }

  /** Multi-line report. */
  std::string report() const {
    std::string out;
    out += fmt::format(
        "{:<26} {:>7} {:>7} {:>7} {:>5} {:>7} {:>16} {:>16} {:>7}\n", "joint",
        "Fc[Nm]", "mu", "Fv", "R2", "gRange", "rest raw [Nm]", "rest comp [Nm]",
        "sigma");
    for (const auto& r : results_) {
      if (!r.identified) {
        out += fmt::format("{:<26} NOT identifiable: {}\n", r.joint, r.reason);
        continue;
      }
      out += fmt::format(
          "{:<26} {:>7.3f} {:>7.3f} {:>7.3f} {:>5.2f} {:>7.2f} "
          "{:>+7.2f}+-{:<7.2f} {:>+7.2f}+-{:<7.2f} {:>7.3f}\n",
          r.joint, r.Fc, r.mu, r.Fv, r.r2, r.gRange, r.restRawMean, r.restRawSd,
          r.restCompMean, r.restCompSd, r.sigma);
    }
    return out;
  }

  /** The yaml block the observer reads. */
  std::string yaml() const {
    std::time_t t = std::time(nullptr);
    char date[32];
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M", std::localtime(&t));
    std::string out;
    out += fmt::format(
        "# Actuator friction model identified by MCControlG1Revo2 --calib on "
        "{}\n",
        date);
    out +=
        "# tau_meas = tau_joint + s*(Fc + mu*|g|) + Fv*qdot ; sigma = RMS "
        "residual at rest\n";
    out +=
        "# Re-run after wear or a large temperature change. Loaded by "
        "mc_external_forces_observer\n";
    out +=
        "# for every controller running this robot (mc_rtc user robot-specific "
        "observer config).\n";
    out += "friction_model:\n";
    out += fmt::format("  velocity_threshold: {:.3f}\n", vs_);
    out += "  band_gain: 1.0\n";
    out += "  joints:\n";
    for (const auto& r : results_) {
      if (!r.identified) {
        continue;
      }
      out += fmt::format(
          "    {}: {{Fc: {:.4f}, mu: {:.4f}, Fv: {:.4f}, sigma: {:.4f}}}\n",
          r.joint, r.Fc, r.mu, r.Fv, r.sigma);
    }
    return out;
  }

  bool write(const std::string& path) const {
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path(), ec);
    std::ofstream f(path);
    if (!f) {
      return false;
    }
    f << yaml();
    return static_cast<bool>(f);
  }

 private:
  static constexpr double kSetupSpeed = 0.3;  // rad/s for moves between poses
  struct Sample {
    double tau, g, dq, q;
  };
  struct Phase {
    Vec from, to;
    double duration;
    int joint;  // tested joint (motor index) or -1
    bool record;
    std::string label;
  };
  void addMove(const Vec& from, const Vec& to, double speed, int joint,
               bool record, const std::string& label) {
    const double dmax = static_cast<double>((to - from).cwiseAbs().maxCoeff());
    const double duration = std::max(dmax / std::max(speed, 1e-3), 0.5);
    phases_.push_back({from, to, duration, joint, record, label});
  }
  void addHold(const Vec& at, double duration, int joint, bool record) {
    phases_.push_back({at, at, duration, joint, record, "hold"});
  }

  double dt_;
  std::array<std::string, N> names_;
  std::map<std::string, int> index_;
  Vec qLimLower_, qLimUpper_, qInit_, qRef_;
  std::vector<Phase> phases_;
  int phase_ = 0;
  double tPhase_ = 0.0, tElapsed_ = 0.0, totalDuration_ = 0.0;
  bool active_ = false;
  int lastLoggedJoint_ = -2;
  std::map<int, std::vector<Sample>> samples_;
  std::vector<JointResult> results_;
  double vs_ = 0.02;
};

}  // namespace mc_unitree
