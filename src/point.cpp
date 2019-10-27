namespace spline{

    point operator*(const point& p1, const point& p2){
        return {p1.x * p2.x, p1.y * p2.y};
    }

    point operator+(const point& p1, const point& p2){
        return {p1.x + p2.x, p1.y + p2.y};
    }

    point operator-(const point& p1, const point& p2){
        return {p1.x - p2.x, p1.y - p2.y};
    }

    point operator*(const point& p, const float& f){
        return {p.x*f, p.y*f};
    }

    point operator*(const float& f, const point& p){
        return {p.x*f, p.y*f};
    }

    point operator/(const point& p, const float& f){
        return {p.x*f, p.y*f};
    }

    point operator/(const float& f, const point& p){
        return {p.x*f, p.y*f};
    }

    float point::magnitude(point p){
        return sqrt( (p.x*p.x) + (p.y*p.y) );
    }

    float point::distance(const point& p1, const point& p2){
        return  magnitude(p1 - p2);
    }

}