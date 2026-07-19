#pragma once
#define BIT_VAL(x) rpl_bit(x)
// ===== Small utilities =====
#define RPL_UNUSED(x) (void)(x)
#define RPL_NO_OP()   (void)0

// Bit value helper (handles enums cleanly)
template <class E>
constexpr unsigned rpl_bit(E x) {
    return 1u << static_cast<unsigned>(x);
}

// Early-return helpers (simple, explicit)
#define RPL_RETURN_IF(cond)          do { if (cond) return; } while (0)
#define RPL_RETURN_VAL_IF(cond, v)   do { if (cond) return (v); } while (0)

// ===== Reusable singleton mixin =====
template <typename Derived>
class crOnceInstance {
public:
    // Access the single instance (created on first use)
    static Derived& instance() {
        static Derived inst;
        return inst;
    }

    crOnceInstance(const crOnceInstance&) = delete;
    crOnceInstance& operator=(const crOnceInstance&) = delete;

protected:
    crOnceInstance() = default;
    ~crOnceInstance() = default;
};

// If your Derived has private ctor/dtor, friend the base:
#define RPL_AUTO_SINGLETON(DerivedT) \
private:                             \
    friend class crOnceInstance<DerivedT>;