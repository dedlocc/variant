#pragma once

#include <array>
#include <stdexcept>
#include <type_traits>
#include <utility>

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

    constexpr trivially_destructible_wrapper<First> &get(in_place_index_t<0>)
    {
        return first;
    }

    constexpr trivially_destructible_wrapper<First> const &get(in_place_index_t<0>) const
    {
        return first;
    }
};

template <typename T, std::size_t... Sizes>
struct multiarray
{
    T t;

    constexpr T at() const noexcept
    {
        return t;
    }
};

template <typename T, std::size_t I, std::size_t... Sizes>
struct multiarray<T, I, Sizes...>
{
    multiarray<T, Sizes...> arr[I], npos;

    constexpr T at(std::size_t i0, auto... is) const noexcept
    {
        return (i0 == variant_npos ? npos : arr[i0]).at(is...);
    }
};

template <std::size_t... Is>
struct seq_first : std::type_identity<std::index_sequence<>>
{};

template <typename Seq>
struct seq_append_npos;

template <std::size_t... Is>
struct seq_append_npos<std::index_sequence<Is...>> : std::type_identity<std::index_sequence<Is..., variant_npos>>
{};

template <std::size_t First, std::size_t... Rest>
struct seq_first<First, Rest...> : seq_append_npos<std::make_index_sequence<First>>
{};

template <std::size_t... Is>
using seq_first_t = typename seq_first<Is...>::type;

template <typename Seq, typename Bound, typename Sizes, typename Foo>
struct visitor_table;

template <std::size_t... Is, std::size_t... Bound, std::size_t Size, std::size_t... Sizes, typename Foo>
struct visitor_table<std::index_sequence<Is...>, std::index_sequence<Bound...>, std::index_sequence<Size, Sizes...>, Foo>
{
    static constexpr multiarray<Foo, Size, Sizes...> table() noexcept
    {
        return {
            visitor_table<seq_first_t<Sizes...>, std::index_sequence<Bound..., Is>, std::index_sequence<Sizes...>, Foo>::table()...
        };
    };
};

template <std::size_t... Bound, typename Result, typename Visitor>
struct visitor_table<std::index_sequence<>, std::index_sequence<Bound...>, std::index_sequence<>, Result (*)(Visitor)>
{
    using Foo = Result (*)(Visitor);

    static constexpr Foo table() noexcept
    {
        return [](Visitor vis) {
            return std::forward<Visitor>(vis)(std::integral_constant<std::size_t, Bound>()...);
        };
    };
};

template <typename>
inline constexpr std::size_t indifferent_zero = 0;

template <typename Visitor, typename... Variants>
inline constexpr auto visitor_table_instance = visitor_table<
    seq_first_t<variant_size_v<std::remove_cvref_t<Variants>>...>,
    std::index_sequence<>,
    std::index_sequence<variant_size_v<std::remove_cvref_t<Variants>>...>,
    std::invoke_result_t<Visitor, std::integral_constant<std::size_t, indifferent_zero<Variants>>...> (*)(Visitor)
>::table();

template <typename Visitor, typename... Variants>
constexpr decltype(auto) visit_indexed(Visitor &&vis, Variants &&...vars)
{
    return visitor_table_instance<Visitor, Variants...>.at(vars.index()...)(std::forward<Visitor>(vis));
}

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
    return detail::visit_indexed([&](auto ...is) -> std::invoke_result_t<Visitor, decltype(get<0>(std::declval<Variants>()))...> {
        if constexpr (((is == variant_npos) || ...)) {
            throw bad_variant_access();
        } else {
            return std::forward<Visitor>(vis)(get<is>(std::forward<Variants>(vars))...);
        }
    }, std::forward<Variants>(vars)...);
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
        detail::visit_indexed([&](auto i) {
            if constexpr (i == variant_npos) {
                this->_index = i;
            } else {
                emplace<i>(get<i>(other));
            }
        }, other);
    }

    constexpr variant(variant &&other) noexcept((std::is_nothrow_move_constructible_v<Types> && ...))
    requires detail::trivially_move_constructible<Types...>
    = default;

    constexpr variant(variant &&other) noexcept((std::is_nothrow_move_constructible_v<Types> && ...))
    requires detail::move_constructible<Types...>
    {
        detail::visit_indexed([&](auto i) {
            if constexpr (i == variant_npos) {
                this->_index = i;
            } else {
                emplace<i>(std::move(get<i>(other)));
            }
        }, other);
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
            this->_storage.get(in_place_index<I>).get() = std::forward<T>(t);
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
        this->_index = variant_npos;

        auto res = new (&this->_storage.get(in_place_index<I>)) detail::trivially_destructible_wrapper<T>(std::forward<Args>(args)...);
        this->_index = I;

        return res->get();
    }

    constexpr void swap(variant &other)
        noexcept(((std::is_nothrow_move_constructible_v<Types> && std::is_nothrow_swappable_v<Types>) && ...))
    {
        detail::visit_indexed([&](auto lhs, auto rhs) {
            if constexpr (lhs == variant_npos) {
                if constexpr (rhs != variant_npos) {
                    emplace<rhs>(std::move(get<rhs>(other)));
                }
            } else if constexpr (rhs == variant_npos) {
                other.template emplace<lhs>(std::move(get<lhs>(*this)));
            } else if constexpr (lhs == rhs) {
                using std::swap;
                swap(get<lhs>(*this), get<rhs>(other));
            } else {
                variant tmp = std::move(other);
                other = std::move(*this);
                *this = std::move(tmp);
            }
            this->_index = rhs;
            other._index = lhs;
        }, *this, other);
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
        return std::move(get<T>(v));
    }

    template <typename T>
    friend constexpr T const &get(variant const &v)
    {
        return get<T>(const_cast<variant &>(v));
    }

    template <typename T>
    friend constexpr T const &&get(variant const &&v)
    {
        return std::move(get<T>(v));
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> &get(variant &v)
    {
        return v.index() == I ? v._storage.get(in_place_index<I>).get() : throw bad_variant_access();
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> &&get(variant &&v)
    {
        return std::move(get<I>(v));
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> const &get(variant const &v)
    {
        return get<I>(const_cast<variant &>(v));
    }

    template <std::size_t I>
    friend constexpr variant_alternative_t<I, variant> const &&get(variant const &&v)
    {
        return std::move(get<I>(v));
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
        return pv && pv->index() == I ? std::addressof(pv->_storage.get(in_place_index<I>).get()) : nullptr;
    }

    template <std::size_t I>
    friend constexpr std::add_pointer_t<const variant_alternative_t<I, variant>> get_if(variant const *pv) noexcept
    {
        return get_if<I>(const_cast<variant *>(pv));
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
