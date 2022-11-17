#include <map>

namespace math{
    template <typename Key, typename Value>
    class InterpolatingMap {
        public:
        /**
        * Inserts a key-value pair.
        *
        * @param key   The key.
        * @param value The value.
        */
        void insert(const Key& key, const Value& value) {
            m_container.insert(std::make_pair(key, value));
        }

        /**
        * Inserts a key-value pair.
        *
        * @param key   The key.
        * @param value The value.
        */
        void insert(Key&& key, Value&& value) {
            m_container.insert(std::make_pair(key, value));
        }

        /**
        * Returns the value associated with a given key.
        *
        * If there's no matching key, the value returned will be a linear
        * interpolation between the keys before and after the provided one.
        *
        * @param key The key.
        */
        Value operator[](const Key& key) const {
            using const_iterator = typename std::map<Key, Value>::const_iterator;
            // Get iterator to upper bound key-value pair for the given key
            const_iterator upper = m_container.upper_bound(key);
            // If key > largest key in table, return value for largest table key
            if (upper == m_container.end()) {
            return (--upper)->second;
            }
            // If key <= smallest key in table, return value for smallest table key
            if (upper == m_container.begin()) {
            return upper->second;
            }
            // Get iterator to lower bound key-value pair
            const_iterator lower = upper;
            --lower;
            // Perform linear interpolation between lower and upper bound
            const double delta = (key - lower->first) / (upper->first - lower->first);
            return delta * upper->second + (1.0 - delta) * lower->second;
        }

        /**
        * Clears the contents.
        */
        void clear() { m_container.clear(); }

        double size(){return m_container.size();}

        private:
        std::map<Key, Value> m_container;
    };
};
