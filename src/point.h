namespace spline{

    class point{
        public:
            point() = default;
            ~point() = default;

            float magnitude(point p);
            float distance(const point& p1, const point& p2);

        private:
            float x;
            float y;
    }

    point operator*(const point& p1, const point& p2);
    point operator+(const point& p1, const point& p2);
    point operator-(const point& p1, const point& p2);
    point operator*(const point& p, const float& f);
    point operator*(const float& f, const point& p);
    point operator/(const point& p, const float& f);
    point operator/(const float& f, const point& p);

    using PointsContainer = std::vector<const point>;
    
}