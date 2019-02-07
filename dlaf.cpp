#include <boost/function_output_iterator.hpp>
#include <boost/geometry/geometry.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

// number of dimensions (must be 2 or 3)
const int D = 2;

// boost is used for its spatial index
typedef boost::geometry::model::point<
    double, D, boost::geometry::cs::cartesian> BoostPoint;

typedef std::pair<BoostPoint, int> IndexValue;

typedef boost::geometry::index::rtree<
    IndexValue, boost::geometry::index::linear<4>> Index;

// Vector represents a point or a vector
class Vector {
public:
    Vector() :
        m_X(0), m_Y(0), m_Z(0) {}

    Vector(double x, double y) :
        m_X(x), m_Y(y), m_Z(0) {}

    Vector(double x, double y, double z) :
        m_X(x), m_Y(y), m_Z(z) {}

    double X() const {
        return m_X;
    }

    double Y() const {
        return m_Y;
    }

    double Z() const {
        return m_Z;
    }

    BoostPoint ToBoost() const {
        return BoostPoint(m_X, m_Y, m_Z);
    }

    double Length() const {
        return std::sqrt(m_X * m_X + m_Y * m_Y + m_Z * m_Z);
    }

    double LengthSquared() const {
        return m_X * m_X + m_Y * m_Y + m_Z * m_Z;
    }

    double Distance(const Vector &v) const {
        const double dx = m_X - v.m_X;
        const double dy = m_Y - v.m_Y;
        const double dz = m_Z - v.m_Z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Vector Normalized() const {
        const double m = 1 / Length();
        return Vector(m_X * m, m_Y * m, m_Z * m);
    }

    Vector operator+(const Vector &v) const {
        return Vector(m_X + v.m_X, m_Y + v.m_Y, m_Z + v.m_Z);
    }

    Vector operator-(const Vector &v) const {
        return Vector(m_X - v.m_X, m_Y - v.m_Y, m_Z - v.m_Z);
    }

    Vector operator*(const double a) const {
        return Vector(m_X * a, m_Y * a, m_Z * a);
    }

    Vector &operator+=(const Vector &v) {
        m_X += v.m_X; m_Y += v.m_Y; m_Z += v.m_Z;
        return *this;
    }

private:
    double m_X;
    double m_Y;
    double m_Z;
};

// Random returns a uniformly distributed random number between lo and hi
double Random(const double lo = 0, const double hi = 1) {
    static thread_local std::mt19937 gen(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(gen);
}

// RandomInUnitSphere returns a random, uniformly distributed point inside the
// unit sphere (radius = 1)
Vector RandomInUnitSphere() {
    while (true) {
        const Vector p = Vector(
            Random(-1, 1),
            Random(-1, 1),
            D == 2 ? 0 : Random(-1, 1));
        if (p.LengthSquared() < 1) {
            return p;
        }
    }
}

// Model holds all of the particles
class Model {
public:
    Model(double particleSpacing, double attractionDistance, double minMoveDistance) :
        m_ParticleSpacing(particleSpacing),
        m_AttractionDistance(attractionDistance),
        m_MinMoveDistance(minMoveDistance),
        m_BoundingRadius(0) {}

    // Add adds a new particle with the specified parent particle
    void Add(const Vector &p, const int parent = -1) {
        const int id = m_Points.size();
        m_Index.insert(std::make_pair(p.ToBoost(), id));
        m_Points.push_back(p);
        m_BoundingRadius = std::max(
            m_BoundingRadius, p.Length() + m_AttractionDistance);
        std::cout
            << id << "," << parent << ","
            << p.X() << "," << p.Y() << "," << p.Z() << std::endl;
    }

    // Nearest returns the index of the particle nearest the specified point
    int Nearest(const Vector &point) const {
        int result = -1;
        m_Index.query(
            boost::geometry::index::nearest(point.ToBoost(), 1),
            boost::make_function_output_iterator([&result](const auto &value) {
                result = value.second;
            }));
        return result;
    }

    // RandomStartingPosition returns a random point to start a new particle
    Vector RandomStartingPosition() const {
        const double d = m_BoundingRadius * 2;
        return RandomInUnitSphere().Normalized() * d;
    }

    // ShouldReset returns true if the particle has gone too far away and
    // should be reset to a new random starting position
    bool ShouldReset(const Vector &p) const {
        return p.Length() > m_BoundingRadius * 4;
    }

    // PlaceParticle computes the final placement of the particle.
    Vector PlaceParticle(const Vector &parent, Vector p) const {
        const Vector v = (p - parent).Normalized();
        return parent + v * m_ParticleSpacing;
    }

    // AddParticle diffuses one new particle and adds it to the model
    void AddParticle() {
        // compute particle starting location
        Vector p = RandomStartingPosition();

        // do the random walk
        while (true) {
            // get distance to nearest other particle
            const int i = Nearest(p);
            const Vector &parent = m_Points[i];
            const double d = parent.Distance(p);

            // check if close enough to join
            if (d < m_AttractionDistance) {
                // adjust particle position in relation to its parent
                p = PlaceParticle(parent, p);

                // add the point
                Add(p, i);
                return;
            }

            // move randomly
            const double m = std::max(
                m_MinMoveDistance, d - m_AttractionDistance);
            p += RandomInUnitSphere().Normalized() * m;

            // check if particle is too far away, reset if so
            if (ShouldReset(p)) {
                p = RandomStartingPosition();
            }
        }
    }

private:
    // m_ParticleSpacing defines the distance between particles that are
    // joined together
    double m_ParticleSpacing;

    // m_AttractionDistance defines how close together particles must be in
    // order to join together
    double m_AttractionDistance;

    // m_MinMoveDistance defines the minimum distance that a particle will move
    // during its random walk
    double m_MinMoveDistance;

    // m_BoundingRadius defines the radius of the bounding sphere that bounds
    // all of the particles
    double m_BoundingRadius;

    // m_Points stores a list of the particle positions
    std::vector<Vector> m_Points;

    // m_Index is the spatial index used to accelerate nearest neighbor queries
    Index m_Index;
};

int main() {
    // create the model
    Model model(1, 3, 0.1);

    // add seed point(s)
    model.Add(Vector());

    // run diffusion-limited aggregation
    for (int i = 0; i < 100000; i++) {
        model.AddParticle();
    }

    return 0;
}
