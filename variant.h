#pragma once

#include <type_traits>
#include <array>
#include <utility>
#include <stdexcept>

template <typename... Types>
struct variant;

inline constexpr std::size_t variant_npos = -1;

struct bad_variant_access : std::runtime_error
{
    bad_variant_access() : std::runtime_error("Bad variant access")
    {}
};

template <typename Variant>
struct variant_size;

template <typename Variant>
inline constexpr std::size_t variant_size_v = variant_size<Variant>::value;

template <typename... Types>
struct variant_size<variant<Types...>> : std::integral_constant<std::size_t, sizeof...(Types)>
{};

template <typename Variant>
struct variant_size<const Variant> : std::integral_constant<std::size_t, variant_size<Variant>::value>
{};

template <std::size_t I, typename Variant>
struct variant_alternative;

template <std::size_t I, typename Variant>
using variant_alternative_t = typename variant_alternative<I, Variant>::type;

template <typename First, typename... Rest>
struct variant_alternative<0, variant<First, Rest...>> : std::type_identity<First>
{};

template <std::size_t I, typename First, typename... Rest>
struct variant_alternative<I, variant<First, Rest...>> : variant_alternative<I - 1, variant<Rest...>>
{};

template <std::size_t I, typename Variant>
struct variant_alternative<I, const Variant> : std::type_identity<std::add_const_t<variant_alternative_t<I, Variant>>>
{};

template <typename T>
struct in_place_type_t
{
    explicit in_place_type_t() = default;
};

template <typename T>
inline constexpr in_place_type_t<T> in_place_type {};

template <std::size_t I>
struct in_place_index_t
{
    explicit in_place_index_t() = default;
};

template <std::size_t I>
inline constexpr in_place_index_t<I> in_place_index {};


namespace detail
{
template <std::size_t I, typename Variant> requires (I < variant_size_v<Variant>)
using type_by_index = variant_alternative_t<I, Variant>;

template <typename T, typename Variant>
struct find_index : std::integral_constant<std::size_t, 0>
{};

template <typename T, typename Variant>
inline constexpr std::size_t find_index_v = find_index<T, Variant>::value;

template <typename T, typename First, typename... Rest> requires (!std::same_as<T, First>)
struct find_index<T, variant<First, Rest...>> : std::integral_constant<std::size_t, find_index_v<T, variant<Rest...>> + 1>
{};

template <typename T, typename Variant>
struct exactly_one : std::false_type
{};

template <typename First, typename... Rest>
struct exactly_one<First, variant<First, Rest...>> : std::bool_constant<find_index_v<First, variant<Rest...>> == sizeof...(Rest)>
{};

template <typename T, typename First, typename... Rest>
struct exactly_one<T, variant<First, Rest...>> : exactly_one<T, variant<Rest...>>
{};

template <class T, typename Variant>
inline constexpr bool exactly_one_v = exactly_one<T, Variant>::value;
} // detail


template <typename T, typename... Types>
constexpr T &get(variant<Types...> &v);

template <typename T, typename... Types>
constexpr T &&get(variant<Types...> &&v);

template <typename T, typename... Types>
constexpr T const &get(variant<Types...> const &v);

template <typename T, typename... Types>
constexpr T const &&get(variant<Types...> const &&v);

template <std::size_t I, typename... Types>
constexpr variant_alternative_t<I, variant<Types...>> &get(variant<Types...> &v);

template <std::size_t I, typename... Types>
constexpr variant_alternative_t<I, variant<Types...>> &&get(variant<Types...> &&v);

template <std::size_t I, typename... Types>
constexpr variant_alternative_t<I, variant<Types...>> const &get(variant<Types...> const &v);

template <std::size_t I, typename... Types>
constexpr variant_alternative_t<I, variant<Types...>> const &&get(variant<Types...> const &&v);


namespace detail
{
template <typename T>
using arr = T[1];

template <typename T, typename Type>
concept overload_is_considered = requires(T t)
{
    arr<Type> {{std::forward<T>(t)}};
};

template <typename T, typename Type, std::size_t I>
struct overload
{
    static std::integral_constant<std::size_t, I> foo(Type)
    requires overload_is_considered<T, Type>;
};

template <typename Seq, typename T, typename... Ts>
struct overloads;

template <typename T, typename... Types, std::size_t... Is>
struct overloads<std::index_sequence<Is...>, T, Types...> : overload<T, Types, Is> ...
{
    using overload<T, Types, Is>::foo...;
};

template <typename T, typename... Types>
using accepted_index = decltype(overloads<std::index_sequence_for<Types...>, T, Types...>::foo(std::declval<T>()));

template <typename T, typename Variant>
inline constexpr std::size_t accepted_index_v = variant_npos;

template <typename T>
concept valid_typename = requires
{
    typename T;
};

template <typename T, typename... Types>
requires valid_typename<accepted_index<T, Types...>>
inline constexpr std::size_t accepted_index_v<T, variant<Types...>> = accepted_index<T, Types...>::value;

template <typename T>
struct trivially_destructible_wrapper
{
    template <typename... Args>
    explicit constexpr trivially_destructible_wrapper(Args &&...args)
    {
        new (&storage) T(std::forward<Args>(args)...);
    }

    constexpr T &get()
    {
        return reinterpret_cast<T &>(storage);
    }

    constexpr T const &get() const
    {
        return reinterpret_cast<T const &>(storage);
    }

    std::aligned_storage_t<sizeof(T), alignof(T)> storage;
};

template <typename T>
requires (std::is_trivially_destructible_v<T>)
struct trivially_destructible_wrapper<T>
{
    template <typename... Args>
    explicit constexpr trivially_destructible_wrapper(Args &&...args) : storage(std::forward<Args>(args)...)
    {}

    constexpr T &get()
    {
        return storage;
    }

    constexpr T const &get() const
    {
        return storage;
    }

    T storage;
};

template <typename... Types>
union variant_union
{};

template <typename First, typename... Rest>
union variant_union<First, Rest...>
{
    trivially_destructible_wrapper<First> first;
    variant_union<Rest...> rest;

    constexpr variant_union() : rest()
    {}

    constexpr ~variant_union() = default;

    template <std::size_t I, typename... Args>
    constexpr explicit variant_union(in_place_index_t<I>, Args &&...args)
        : rest(in_place_index<I - 1>, std::forward<Args>(args)...)
    {}

    template <typename... Args>
    constexpr explicit variant_union(in_place_index_t<0>, Args &&...args)
        : first(std::forward<Args>(args)...)
    {}


    template <std::size_t I>
    constexpr decltype(auto) get(in_place_index_t<I>)
    {
        return rest.get(in_place_index<I - 1>);
    }

    template <std::size_t I>
    constexpr decltype(auto) get(in_place_index_t<I>) const
    {
        return rest.get(in_place_index<I - 1>);
    }

    constexpr First &get(in_place_index_t<0>)
    {
        return first.get();
    }

    constexpr First const &get(in_place_index_t<0>) const
    {
        return first.get();
    }
};

template <typename... Variants>
struct visitor_size_product : std::integral_constant<std::size_t, 1>
{};

template <typename... Variants>
inline constexpr std::size_t visitor_size_product_v = visitor_size_product<Variants...>::value;

template <typename First, typename... Rest>
struct visitor_size_product<First, Rest...>
    : std::integral_constant<std::size_t, variant_size_v<std::remove_reference_t<First>> * visitor_size_product_v<Rest...>>
{};

template <typename... Variants>
struct visitor_encode_index;

template <typename... Variants>
constexpr std::size_t visitor_encode_index_v(Variants &&...vars)
{
    return visitor_encode_index<Variants...>::foo(std::forward<Variants>(vars)...);
}

template <typename Variant>
struct visitor_encode_index<Variant>
{
    constexpr static std::size_t foo(Variant &&var)
    {
        return var.index();
    }
};

template <typename First, typename... Rest>
struct visitor_encode_index<First, Rest...>
{
    constexpr static std::size_t foo(First &&first, Rest &&...rest)
    {
        return first.index() * visitor_size_product_v<Rest...> + visitor_encode_index_v(std::forward<Rest>(rest)...);
    }
};

template <std::size_t I, std::size_t Skip, std::size_t... VariantSizes>
struct visitor_decode_index;

template <std::size_t I, std::size_t Skip, std::size_t First, std::size_t... Rest>
struct visitor_decode_index<I, Skip, First, Rest...> : visitor_decode_index<I, Skip - 1, Rest...>
{};

template <std::size_t I, std::size_t First, std::size_t... Rest>
struct visitor_decode_index<I, 0, First, Rest...> : visitor_decode_index<I / First, 0, Rest...>
{};

template <std::size_t I, std::size_t First>
struct visitor_decode_index<I, 0, First> : std::integral_constant<std::size_t, I % First>
{};

template <std::size_t I, std::size_t Skip, std::size_t... VariantSizes>
inline constexpr std::size_t visitor_decode_index_v = visitor_decode_index<I, Skip, VariantSizes...>::value;

template <typename Visitor, typename... Variants>
struct visit_result
    : std::type_identity<std::invoke_result_t<Visitor, decltype(get<0>(std::declval<Variants>()))...>>
{};

template <typename Visitor, typename... Variants>
using visit_result_t = typename visit_result<Visitor, Variants...>::type;

template <std::size_t I, typename Seq, typename Visitor, typename... Variants>
struct call_visit;

template <std::size_t I, typename Visitor, typename... Variants, std::size_t... Is>
struct call_visit<I, std::index_sequence<Is...>, Visitor, Variants...>
{
    static constexpr decltype(auto) foo(Visitor &&vis, Variants &&...vars)
    {
        return std::forward<Visitor>(vis)(get<visitor_decode_index_v<I, Is, visitor_size_product_v<Variants>...>>(std::forward<Variants>(vars))...);
    }
};

template <typename Seq, typename Visitor, typename... Variants>
struct visitor_table;

template <typename Visitor, typename... Variants, std::size_t... Is>
struct visitor_table<std::index_sequence<Is...>, Visitor, Variants...>
{
    constexpr static std::array<visit_result_t<Visitor, Variants...> (*)(Visitor &&, Variants &&...), sizeof...(Is)> table {
        [](Visitor &&vis, Variants &&...vars) {
            return call_visit<
                Is,
                std::make_index_sequence<sizeof...(Variants)>,
                Visitor,
                Variants...
            >::foo(std::forward<Visitor>(vis), std::forward<Variants>(vars)...);
        }...
    };
};

template <typename... Types>
concept copy_constructible = (std::copy_constructible<Types> && ...);

template <typename... Types>
concept trivially_copy_constructible = copy_constructible<Types...> && (std::is_trivially_copy_constructible_v<Types> && ...);

template <typename... Types>
concept copy_assignable = copy_constructible<Types...> && (std::is_copy_assignable_v<Types> && ...);

template <typename... Types>
concept trivially_copy_assignable = copy_assignable<Types...>
    && trivially_copy_constructible<Types...>
    && (std::is_trivially_copy_assignable_v<Types> && ...)
    && (std::is_trivially_destructible_v<Types> && ...);

template <typename... Types>
concept move_constructible = (std::move_constructible<Types> && ...);

template <typename... Types>
concept trivially_move_constructible = move_constructible<Types...> && (std::is_trivially_move_constructible_v<Types> && ...);

template <typename... Types>
concept move_assignable = move_constructible<Types...> && (std::is_move_assignable_v<Types> && ...);

template <typename... Types>
concept trivially_move_assignable = move_assignable<Types...>
    && trivially_move_constructible<Types...>
    && (std::is_trivially_move_assignable_v<Types> && ...)
    && (std::is_trivially_destructible_v<Types> && ...);

template <typename To, typename From>
struct copy_c_ref;

template <typename To, typename From>
struct copy_c_ref<To, From &> : std::type_identity<To &>
{};

template <typename To, typename From>
struct copy_c_ref<To, From &&> : std::type_identity<To &&>
{};

template <typename To, typename From>
struct copy_c_ref<To, From const &> : std::type_identity<To const &>
{};

template <typename To, typename From>
struct copy_c_ref<To, From const &&> : std::type_identity<To const &&>
{};

template <typename To, typename From>
using copy_c_ref_t = typename copy_c_ref<To, From>::type;

template <typename... Types>
struct variant_base
{
    using Variant = variant<Types...>;

    constexpr variant_base() = default;

    template <std::size_t I, typename... Args, typename Tj = detail::type_by_index<I, Variant>>
    requires std::constructible_from<Tj, Args...>
    constexpr explicit variant_base(in_place_index_t<I>, Args &&...args)
        : _index(I)
        , _storage(in_place_index<I>, std::forward<Args>(args)...)
    {}

    constexpr bool valueless_by_exception() const noexcept
    {
        return index() == variant_npos;
    }

    constexpr std::size_t index() const noexcept
    {
        return _index;
    }

protected:
    std::size_t _index = variant_npos;
    detail::variant_union<Types...> _storage;

    constexpr void _destroy()
    {
        if (!this->valueless_by_exception()) {
            visit([]<typename T>(T &item) {
                item.~T();
            }, static_cast<copy_c_ref_t<Variant, decltype(*this)>>(*this));
        }
    }
};

template <typename... Types>
struct variant_destructor_base : variant_base<Types...>
{
    using variant_base<Types...>::variant_base;

    constexpr ~variant_destructor_base()
    {
        this->_destroy();
    }
};

template <typename... Types>
requires (std::is_trivially_destructible_v<Types> && ...)
struct variant_destructor_base<Types...> : variant_base<Types...>
{
    using variant_base<Types...>::variant_base;
};

} // detail

template <typename Visitor, typename... Variants>
constexpr decltype(auto) visit(Visitor &&vis, Variants &&...vars)
{
    if ((vars.valueless_by_exception() || ...)) {
        throw bad_variant_access();
    }
    using Seq = std::make_index_sequence<detail::visitor_size_product_v<Variants...>>;
    std::size_t index = detail::visitor_encode_index_v(std::forward<Variants>(vars)...);
    return detail::visitor_table<Seq, Visitor, Variants...>::table[index](std::forward<Visitor>(vis), std::forward<Variants>(vars)...);
}

template <typename R, typename Visitor, typename... Variants>
constexpr R visit(Visitor &&vis, Variants &&...vars)
{
    return visit(std::forward<Visitor>(vis), std::forward<Variants>(vars)...);
}

template <typename... Types>
struct variant : detail::variant_destructor_base<Types...>
{
private:
    using Base = detail::variant_destructor_base<Types...>;

public:
    constexpr variant() noexcept(std::is_nothrow_default_constructible_v<variant_alternative_t<0, variant>>)
    requires (std::is_default_constructible_v<detail::type_by_index<0, variant>>)
        : Base(in_place_index<0>)
    {}

    constexpr variant(variant const &other)
    requires detail::trivially_copy_constructible<Types...>
    = default;

    constexpr variant(variant const &other)
    requires detail::copy_constructible<Types...>
    {
        this->_index = other._index;
        if (!this->valueless_by_exception()) {
            visit([this]<typename T>(T &item) {
                new (&this->_storage) T(item);
            }, other);
        }
    }

    constexpr variant(variant &&other) noexcept((std::is_nothrow_move_constructible_v<Types> && ...))
    requires detail::trivially_move_constructible<Types...>
    = default;

    constexpr variant(variant &&other) noexcept((std::is_nothrow_move_constructible_v<Types> && ...))
    requires detail::move_constructible<Types...>
    {
        this->_index = other._index;
        if (!this->valueless_by_exception()) {
            visit([this]<typename T>(T &&item) {
                new (&this->_storage) std::remove_reference_t<T>(std::move(item));
            }, std::move(other));
        }
    }

    template <typename T, typename Tj = detail::type_by_index<detail::accepted_index_v<T &&, variant>, variant>>
    requires std::constructible_from<Tj, T>
    constexpr variant(T &&t) noexcept(std::is_nothrow_constructible_v<Tj, T>)
        : variant(in_place_type<Tj>, std::forward<T>(t))
    {}

    template <typename T, typename... Args>
    requires std::constructible_from<T, Args...> && detail::exactly_one_v<T, variant>
    constexpr explicit variant(in_place_type_t<T>, Args &&...args)
        : variant(in_place_index<detail::find_index_v<T, variant>>, std::forward<Args>(args)...)
    {}

    template <std::size_t I, typename... Args, typename Tj = detail::type_by_index<I, variant>>
    requires std::constructible_from<Tj, Args...>
    constexpr explicit variant(in_place_index_t<I>, Args &&...args)
        : Base(in_place_index<I>, std::forward<Args>(args)...)
    {}

    constexpr variant &operator=(variant const &other)
    requires detail::trivially_copy_assignable<Types...>
    = default;

    constexpr variant &operator=(variant const &other)
    requires detail::copy_assignable<Types...>
    {
        variant tmp = other;
        swap(tmp);
        return *this;
    }

    constexpr variant &operator=(variant &&other)
    requires detail::trivially_move_assignable<Types...>
    = default;

    constexpr variant &operator=(variant &&other)
        noexcept(((std::is_nothrow_move_constructible_v<Types> && std::is_nothrow_move_assignable_v<Types>) && ...))
    requires detail::move_assignable<Types...>
    {
        if (other.valueless_by_exception()) {
            this->_destroy();
        }

        visit([&]<typename L, typename R>(L &lhs, R &rhs) {
            if constexpr (std::is_same_v<L, R>) {
                lhs = std::move(rhs);
            } else {
                emplace<R>(std::move(rhs));
            }
        }, *this, other);

        this->_index = other.index();
        return *this;
    }

    template <typename T, std::size_t I = detail::accepted_index_v<T, variant>, typename Tj = detail::type_by_index<I, variant>>
    requires std::constructible_from<Tj, T>
    constexpr variant &operator=(T &&t)
        noexcept(std::is_nothrow_assignable_v<Tj&, T> && std::is_nothrow_constructible_v<Tj, T>)
    {
        if (this->index() == I) {
            this->_storage.get(in_place_index<I>) = std::forward<T>(t);
        } else {
            emplace<I>(Tj(std::forward<T>(t)));
        }
        return *this;
    }

    template <typename T, typename... Args>
    requires std::constructible_from<T, Args...>
    constexpr T &emplace(Args &&...args)
    {
        return emplace<detail::find_index_v<T, variant>>(std::forward<Args>(args)...);
    }

    template <std::size_t I, typename... Args, typename T = detail::type_by_index<I, variant>>
    requires std::constructible_from<T, Args...>
    constexpr variant_alternative_t<I, variant> &emplace(Args &&...args)
    {
        this->_destroy();
        this->_index = I;
        try {
            return *new(&this->_storage) T(std::forward<Args>(args)...);
        } catch (...) {
            this->_index = variant_npos;
            throw;
        }
    }

    constexpr void swap(variant &other)
        noexcept(((std::is_nothrow_move_constructible_v<Types> && std::is_nothrow_swappable_v<Types>) && ...))
    {
        auto index_left = this->index();
        auto index_right = other.index();

        auto valueless_left = this->valueless_by_exception();
        auto valueless_right = other.valueless_by_exception();

        if (valueless_left) {
            if (!valueless_right) {
                visit([&]<typename T>(T &item) {
                    emplace<T>(std::move(item));
                }, other);
                other._destroy();
            }
        } else if (valueless_right) {
            visit([&]<typename T>(T &item) {
                other.template emplace<T>(std::move(item));
            }, *this);
            this->_destroy();
        } else {
            visit([&]<typename L, typename R>(L &lhs, R &rhs) {
                if constexpr (std::is_same_v<L, R>) {
                    using std::swap;
                    swap(lhs, rhs);
                } else {
                    try {
                        auto tmp = std::move(rhs);
                        other.template emplace<L>(std::move(lhs));
                        emplace<R>(std::move(tmp));
                    } catch (...) {
                        this->_destroy();
                        this->_index = variant_npos;
                        throw;
                    }
                }
            }, *this, other);
        }

        this->_index = index_right;
        other._index = index_left;
    }

    template <typename T>
    friend constexpr bool holds_alternative(variant const &v) noexcept
    {
        return detail::find_index_v<T, variant> == v.index();
    }

    template <typename T>
    friend constexpr T &get(variant &v)
    {
        return get<detail::find_index_v<T, variant>>(v);
    }

    template <typename T>
    friend constexpr T &&get(variant &&v)
    {
        return get<detail::find_index_v<T, variant>>(v);
    }

    template <typename T>
    friend constexpr T const &get(variant const &v)
    {
        return get<detail::find_index_v<T, variant>>(v);
    }

    template <typename T>
    friend constexpr T const &&get(variant const &&v)
    {
        return get<detail::find_index_v<T, variant>>(v);
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> &get(variant &v)
    {
        if (v.index() == I) {
            return v._storage.get(in_place_index<I>);
        }
        throw bad_variant_access();
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> &&get(variant &&v)
    {
        if (v.index() == I) {
            return std::move(v._storage.get(in_place_index<I>));
        }
        throw bad_variant_access();
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> const &get(variant const &v)
    {
        if (v.index() == I) {
            return v._storage.get(in_place_index<I>);
        }
        throw bad_variant_access();
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> const &&get(variant const &&v)
    {
        if (v.index() == I) {
            return std::move(v._storage.get(in_place_index<I>));
        }
        throw bad_variant_access();
    }

    template <typename T>
    friend constexpr std::add_pointer_t<T> get_if(variant *pv) noexcept
    {
        return get_if<detail::find_index_v<T, variant>>(pv);
    }

    template <typename T>
    friend constexpr std::add_pointer_t<const T> get_if(variant const *pv) noexcept
    {
        return get_if<detail::find_index_v<T, variant>>(pv);
    }

    template <std::size_t I>
    friend constexpr std::add_pointer_t<variant_alternative_t<I, variant>> get_if(variant *pv) noexcept
    {
        return pv && pv->index() == I ? std::addressof(pv->_storage.get(in_place_index<I>)) : nullptr;
    }

    template <std::size_t I>
    friend constexpr std::add_pointer_t<const variant_alternative_t<I, variant>> get_if(variant const *pv) noexcept
    {
        return pv && pv->index() == I ? std::addressof(pv->_storage.get(in_place_index<I>)) : nullptr;
    }

    friend constexpr bool operator==(variant const &lhs, variant const &rhs)
    {
        return (lhs <=> rhs) == std::strong_ordering::equal;
    }

    friend constexpr std::strong_ordering operator<=>(variant const &lhs, variant const &rhs)
    {
        auto valueless_left = lhs.valueless_by_exception();
        auto valueless_right = rhs.valueless_by_exception();

        if (valueless_left && valueless_right) {
            return std::strong_ordering::equal;
        }
        if (valueless_left) {
            return std::strong_ordering::less;
        }
        if (valueless_right) {
            return std::strong_ordering::greater;
        }
        if (lhs.index() != rhs.index()) {
            return lhs.index() <=> rhs.index();
        }
        return visit([](auto &&l, auto &&r) {
            if constexpr (std::is_same_v<decltype(l), decltype(r)>) {
                if (l < r) {
                    return std::strong_ordering::less;
                }
                if (r < l) {
                    return std::strong_ordering::greater;
                }
            }
            return std::strong_ordering::equal;
        }, lhs, rhs);
    }
};
