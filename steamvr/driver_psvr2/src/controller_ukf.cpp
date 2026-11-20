// SPDX-License-Identifier: GPL-2.0
// Controller pose filter; see controller_ukf.h and docs/sony-re/ukf.md.
#include "controller_ukf.h"

#include <algorithm>
#include <cmath>

using vrmath::Vec3;

namespace
{

// Error-state block offsets.
constexpr int kP = 0, kV = 3, kTh = 6, kBg = 9, kBa = 12;
constexpr int N = ControllerUkf::N;
constexpr int kMaxM = 12;

constexpr double kGravity = 9.80665;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

// Sony's values (docs/sony-re/ukf.md, tracking.md 5.5, opticalpose.md 4).
constexpr double kP0Pos = 0.0025, kP0Vel = 0.04;
constexpr double kP0Att[3] = { 1.0 * kDeg * 1.0 * kDeg, 5.0 * kDeg * 5.0 * kDeg, 1.0 * kDeg * 1.0 * kDeg };
constexpr double kGyroBase = 3.0e-4, kGyroOwn = 2.5e-5, kGyroCross = 6.25e-6, kGyroFixed = 3.04617;
constexpr double kGyroRange = 43.6332;  // rad/s (2500 deg/s)
constexpr double kAccBase = 8.0e-6, kAccExtra = 1.0e-4, kAccOwn = 2.5e-5, kAccCross = 6.25e-6, kAccFixed = 6.25;
constexpr double kGateNear = 0.05, kGateFar = 0.20, kGateSpeed = 1.0;
constexpr double kStillAccel = 1.96, kStillDamp = 0.99;
constexpr uint64_t kStaleUs = 300000;  // optical considered stale (opticalpose.md 5)
constexpr double kVelLimit = 32.0, kGyroBiasLimit = 0.34907, kAccBiasLimit = 2.0;  // [likely] limits
constexpr double kZone[3] = { 0.30, 0.20, 0.25 };  // head ellipsoid semi-axes, m
constexpr double kZoneBand = 0.04;
constexpr double kSigmaPosOut = 0.0005, kSigmaPosIn = 0.0025, kSigmaPosFar = 0.01, kFarDist = 1.1;
constexpr double kSigmaPosOnly = 0.05;
constexpr double kTip = 0.1;  // body-frame axis length, m

// Own values.
constexpr double kAccRange = 6.0 * kGravity;  // [guess] accel "fixed" noise threshold
constexpr double kBgWalk = 1.9038590e-7, kBaWalk = 1.0e-6;  // per second [guess]
constexpr double kP0Bg = 1.0e-4, kP0Ba = 0.04;
constexpr double kMaxGap = 0.5;     // s of missing IMU before a reset
constexpr uint64_t kKeepUs = 1000000;  // IMU history
constexpr int kMaxRejects = 10;
constexpr double kGateSigmas = 3.0;

Vec3 Sub( const Vec3 &a, const Vec3 &b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
double Dot( const Vec3 &a, const Vec3 &b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double Norm( const Vec3 &a ) { return std::sqrt( Dot( a, a ) ); }
double At( const Vec3 &v, int i ) { return i == 0 ? v.x : i == 1 ? v.y : v.z; }
Vec3 FromArr( const double *a ) { return { a[0], a[1], a[2] }; }

// Rotation increment exp(w): w in rad (body frame).
vr::HmdQuaternion_t Exp( const Vec3 &w )
{
	const double a = Norm( w );
	const double h = 0.5 * a;
	const double k = a < 1e-9 ? 0.5 : std::sin( h ) / a;
	return { std::cos( h ), w.x * k, w.y * k, w.z * k };
}

// R[r][c], world <- body.
void Rot( const vr::HmdQuaternion_t &q, double R[3][3] )
{
	const double w = q.w, x = q.x, y = q.y, z = q.z;
	R[0][0] = 1 - 2 * ( y * y + z * z );
	R[0][1] = 2 * ( x * y - w * z );
	R[0][2] = 2 * ( x * z + w * y );
	R[1][0] = 2 * ( x * y + w * z );
	R[1][1] = 1 - 2 * ( x * x + z * z );
	R[1][2] = 2 * ( y * z - w * x );
	R[2][0] = 2 * ( x * z - w * y );
	R[2][1] = 2 * ( y * z + w * x );
	R[2][2] = 1 - 2 * ( x * x + y * y );
}

Vec3 MulR( const double R[3][3], const Vec3 &v )
{
	return { R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z, R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
	         R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z };
}

// In-place lower Cholesky of an n x n matrix (row stride ld). False if not
// positive definite.
bool Cholesky( double *A, int n, int ld )
{
	for ( int j = 0; j < n; j++ )
	{
		double d = A[j * ld + j];
		for ( int k = 0; k < j; k++ )
			d -= A[j * ld + k] * A[j * ld + k];
		if ( !( d > 0.0 ) )
			return false;
		d = std::sqrt( d );
		A[j * ld + j] = d;
		for ( int i = j + 1; i < n; i++ )
		{
			double s = A[i * ld + j];
			for ( int k = 0; k < j; k++ )
				s -= A[i * ld + k] * A[j * ld + k];
			A[i * ld + j] = s / d;
		}
		for ( int k = j + 1; k < n; k++ )
			A[j * ld + k] = 0.0;
	}
	return true;
}

// Solve L L^T x = b in place (L from Cholesky, row stride ld).
void CholSolve( const double *L, int n, int ld, double *b )
{
	for ( int i = 0; i < n; i++ )
	{
		double s = b[i];
		for ( int k = 0; k < i; k++ )
			s -= L[i * ld + k] * b[k];
		b[i] = s / L[i * ld + i];
	}
	for ( int i = n - 1; i >= 0; i-- )
	{
		double s = b[i];
		for ( int k = i + 1; k < n; k++ )
			s -= L[k * ld + i] * b[k];
		b[i] = s / L[i * ld + i];
	}
}

// Sony's rate-dependent per-axis variance (ctrltracker.md "Process noise").
void RateNoise( const Vec3 &v, double own, double cross, double base, double fixed, double range, double out[3] )
{
	for ( int i = 0; i < 3; i++ )
	{
		const int j = ( i + 1 ) % 3, k = ( i + 2 ) % 3;
		const double vi = At( v, i ), vj = At( v, j ), vk = At( v, k );
		out[i] = std::fabs( vi ) >= range ? fixed : ( ( vi * vi * own + vj * vj * cross ) + vk * vk * cross ) + base;
	}
}

}  // namespace

void ControllerUkf::Reset()
{
	init_ = false;
	rejects_ = 0;
	last_fix_us_ = 0;
	for ( auto &row : P_ )
		std::fill( std::begin( row ), std::end( row ), 0.0 );
}

void ControllerUkf::Propagate( State &s, const Vec3 &gyro, const Vec3 &accel, double dt )
{
	const Vec3 w = Sub( gyro, s.bg );
	const Vec3 a = Sub( accel, s.ba );
	double Rm[3][3];
	Rot( vrmath::Mul( s.q, Exp( vrmath::Scale( w, 0.5 * dt ) ) ), Rm );
	Vec3 aw = MulR( Rm, a );
	aw.y -= kGravity;
	s.p = vrmath::Add( s.p, vrmath::Add( vrmath::Scale( s.v, dt ), vrmath::Scale( aw, 0.5 * dt * dt ) ) );
	s.v = vrmath::Add( s.v, vrmath::Scale( aw, dt ) );
	s.q = vrmath::Normalize( vrmath::Mul( s.q, Exp( vrmath::Scale( w, dt ) ) ) );
	s.a_prev = aw;
	s.us += uint64_t( std::llround( dt * 1e6 ) );
}

void ControllerUkf::AddImu( uint64_t us, const Vec3 &gyro, const Vec3 &accel )
{
	if ( !ring_.empty() )
	{
		const uint64_t prev = ring_.back().us;
		if ( us <= prev )
			return;
		const double dt = double( us - prev ) * 1e-6;
		if ( dt > kMaxGap )
		{
			Reset();
			ring_.clear();
		}
		else if ( init_ )
		{
			Propagate( cur_, gyro, accel, dt );
			cur_.us = us;
			// Stationary damping (Sony): only on the path without fresh
			// optical updates, where nothing else bounds the velocity.
			if ( us > last_fix_us_ + kStaleUs && Norm( cur_.a_prev ) < kStillAccel )
				cur_.v = vrmath::Scale( cur_.v, kStillDamp );
		}
	}
	if ( !init_ )
		cur_.us = us;
	ring_.push_back( { us, gyro, accel, cur_ } );
	while ( ring_.size() > 2 && ring_.front().us + kKeepUs < us )
		ring_.pop_front();
}

void ControllerUkf::SetAttitudeP0( const vr::HmdQuaternion_t &q )
{
	// Sony's (1, 5, 1) deg is about world x, y, z (yaw loose); our attitude
	// error is body-frame: P = R^T D R.
	double R[3][3];
	Rot( q, R );
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
		{
			double s = 0.0;
			for ( int k = 0; k < 3; k++ )
				s += R[k][i] * kP0Att[k] * R[k][j];
			P_[kTh + i][kTh + j] = s;
		}
}

void ControllerUkf::Init( const Fix &f, const State &at )
{
	Reset();
	State s = at;
	s.p = f.pos;
	s.v = {};
	s.q = vrmath::Normalize( f.rot );
	s.bg = bg_hint_;
	s.ba = {};
	s.a_prev = {};
	cur_ = s;
	for ( int i = 0; i < 3; i++ )
	{
		P_[kP + i][kP + i] = kP0Pos;
		P_[kV + i][kV + i] = kP0Vel;
		P_[kBg + i][kBg + i] = kP0Bg;
		P_[kBa + i][kBa + i] = kP0Ba;
	}
	SetAttitudeP0( s.q );
	p_us_ = s.us;
	init_ = true;
}

void ControllerUkf::ResetBlock( int b, double var )
{
	for ( int i = b; i < b + 3; i++ )
		for ( int j = 0; j < N; j++ )
			P_[i][j] = P_[j][i] = 0.0;
	for ( int i = b; i < b + 3; i++ )
		P_[i][i] = var;
}

bool ControllerUkf::CovarianceStep( uint64_t to_us, const State &s )
{
	// Time-weighted mean IMU sample over (p_us_, to_us]: sample k holds over
	// (t[k-1], t[k]].
	Vec3 gsum, asum;
	double tsum = 0.0;
	for ( size_t k = 1; k < ring_.size(); k++ )
	{
		const uint64_t a = std::max( ring_[k - 1].us, p_us_ ), b = std::min( ring_[k].us, to_us );
		if ( b <= a )
			continue;
		const double w = double( b - a ) * 1e-6;
		gsum = vrmath::Add( gsum, vrmath::Scale( ring_[k].gyro, w ) );
		asum = vrmath::Add( asum, vrmath::Scale( ring_[k].accel, w ) );
		tsum += w;
	}
	const double T = to_us > p_us_ ? double( to_us - p_us_ ) * 1e-6 : 0.0;
	p_us_ = std::max( p_us_, to_us );
	if ( T <= 0.0 )
		return true;
	if ( tsum <= 0.0 )
	{
		gsum = vrmath::Scale( ring_.back().gyro, T );
		asum = vrmath::Scale( ring_.back().accel, T );
		tsum = T;
	}
	const Vec3 graw = vrmath::Scale( gsum, 1.0 / tsum ), araw = vrmath::Scale( asum, 1.0 / tsum );
	const Vec3 w = Sub( graw, s.bg ), a = Sub( araw, s.ba );
	double R[3][3];
	Rot( s.q, R );

	// Continuous error dynamics F, then Phi = I + F T + (F T)^2 / 2.
	double F[N][N], Phi[N][N], tmp[N][N];
	for ( auto &row : F )
		std::fill( std::begin( row ), std::end( row ), 0.0 );
	const double ax[3][3] = { { 0, -a.z, a.y }, { a.z, 0, -a.x }, { -a.y, a.x, 0 } };
	const double wx[3][3] = { { 0, -w.z, w.y }, { w.z, 0, -w.x }, { -w.y, w.x, 0 } };
	for ( int i = 0; i < 3; i++ )
	{
		F[kP + i][kV + i] = 1.0;
		F[kTh + i][kBg + i] = -1.0;
		for ( int j = 0; j < 3; j++ )
		{
			double s2 = 0.0;
			for ( int k = 0; k < 3; k++ )
				s2 += R[i][k] * ax[k][j];
			F[kV + i][kTh + j] = -s2;
			F[kV + i][kBa + j] = -R[i][j];
			F[kTh + i][kTh + j] = -wx[i][j];
		}
	}
	for ( int i = 0; i < N; i++ )
		for ( int j = 0; j < N; j++ )
		{
			double f2 = 0.0;
			for ( int k = 0; k < N; k++ )
				f2 += F[i][k] * F[k][j];
			Phi[i][j] = ( i == j ? 1.0 : 0.0 ) + F[i][j] * T + 0.5 * f2 * T * T;
		}
	// P = Phi P Phi^T.
	for ( int i = 0; i < N; i++ )
		for ( int j = 0; j < N; j++ )
		{
			double s2 = 0.0;
			for ( int k = 0; k < N; k++ )
				s2 += Phi[i][k] * P_[k][j];
			tmp[i][j] = s2;
		}
	for ( int i = 0; i < N; i++ )
		for ( int j = 0; j < N; j++ )
		{
			double s2 = 0.0;
			for ( int k = 0; k < N; k++ )
				s2 += tmp[i][k] * Phi[j][k];
			P_[i][j] = s2;
		}

	// Q: Sony's IMU noise on the mean rate / accel, held over the interval.
	double gv[3], av[3];
	RateNoise( graw, kGyroOwn, kGyroCross, kGyroBase, kGyroFixed, kGyroRange, gv );
	RateNoise( araw, kAccOwn, kAccCross, kAccBase + kAccExtra, kAccFixed, kAccRange, av );
	const double T2 = T * T;
	for ( int i = 0; i < 3; i++ )
	{
		P_[kTh + i][kTh + i] += gv[i] * T2;
		P_[kBg + i][kBg + i] += kBgWalk * T;
		P_[kBa + i][kBa + i] += kBaWalk * T;
		for ( int j = 0; j < 3; j++ )
		{
			double q = 0.0;  // (R diag(av) R^T)_ij
			for ( int k = 0; k < 3; k++ )
				q += R[i][k] * av[k] * R[j][k];
			P_[kV + i][kV + j] += q * T2;
			P_[kP + i][kP + j] += q * T2 * T2 * 0.25;
			P_[kP + i][kV + j] += q * T2 * T * 0.5;
			P_[kV + i][kP + j] += q * T2 * T * 0.5;
		}
	}
	return true;
}

bool ControllerUkf::Update( State &s, const Fix &f )
{
	const int m = f.has_rot ? 12 : 3;

	// Measurement noise (ukf.md 4).
	double sp, sr = 0.0;
	if ( !f.has_rot )
		sp = kSigmaPosOnly;
	else
	{
		const Vec3 &h = f.head_rel;
		const double r = Norm( h );
		double r_s = 1e9;
		if ( r > 1e-9 )
		{
			const double ux = h.x / r / kZone[0], uy = h.y / r / kZone[1], uz = h.z / r / kZone[2];
			r_s = 1.0 / std::sqrt( ux * ux + uy * uy + uz * uz );
		}
		if ( r <= r_s - kZoneBand || r >= r_s + kZoneBand )
			sp = r < r_s ? kSigmaPosIn : kSigmaPosOut;
		else
		{
			const double wgt = ( r_s + kZoneBand - r ) / ( 2.0 * kZoneBand );
			sp = std::sqrt( wgt * kSigmaPosIn * kSigmaPosIn + ( 1.0 - wgt ) * kSigmaPosOut * kSigmaPosOut );
		}
		if ( f.cam_dist > kFarDist )
			sp = kSigmaPosFar;
	}
	sp = std::max( sp, pos_sigma_floor_ );
	sr = std::max( sr, rot_sigma_floor_ );

	// Sigma points over the error state.
	double L[N][N];
	for ( int i = 0; i < N; i++ )
		for ( int j = 0; j < N; j++ )
			L[i][j] = 0.5 * ( P_[i][j] + P_[j][i] );
	if ( !Cholesky( &L[0][0], N, N ) )
	{
		for ( int i = 0; i < N; i++ )
			for ( int j = 0; j < N; j++ )
				L[i][j] = 0.5 * ( P_[i][j] + P_[j][i] ) + ( i == j ? 1e-10 : 0.0 );
		if ( !Cholesky( &L[0][0], N, N ) )
			return false;
	}
	const double spread = std::sqrt( double( N ) ), wgt = 1.0 / ( 2.0 * N );
	double H[2 * N][kMaxM];
	double ybar[kMaxM] = {};
	for ( int pt = 0; pt < 2 * N; pt++ )
	{
		const int col = pt % N;
		const double sg = pt < N ? spread : -spread;
		double d[N];
		for ( int i = 0; i < N; i++ )
			d[i] = sg * L[i][col];
		const Vec3 p = vrmath::Add( s.p, FromArr( d + kP ) );
		double *h = H[pt];
		h[0] = p.x, h[1] = p.y, h[2] = p.z;
		if ( m == 12 )
		{
			double R[3][3];
			Rot( vrmath::Mul( s.q, Exp( FromArr( d + kTh ) ) ), R );
			for ( int c = 0; c < 3; c++ )
				for ( int r = 0; r < 3; r++ )
					h[3 + 3 * c + r] = kTip * R[r][c];
		}
		for ( int i = 0; i < m; i++ )
			ybar[i] += wgt * h[i];
	}
	double S[kMaxM][kMaxM] = {}, C[N][kMaxM] = {};
	for ( int pt = 0; pt < 2 * N; pt++ )
	{
		const int col = pt % N;
		const double sg = pt < N ? spread : -spread;
		double e[kMaxM];
		for ( int i = 0; i < m; i++ )
			e[i] = H[pt][i] - ybar[i];
		for ( int i = 0; i < m; i++ )
			for ( int j = 0; j < m; j++ )
				S[i][j] += wgt * e[i] * e[j];
		for ( int i = 0; i < N; i++ )
		{
			const double di = sg * L[i][col];
			for ( int j = 0; j < m; j++ )
				C[i][j] += wgt * di * e[j];
		}
	}
	for ( int i = 0; i < m; i++ )
		S[i][i] += i < 3 ? sp * sp : sr * sr;

	// Innovation.
	double z[kMaxM];
	z[0] = f.pos.x, z[1] = f.pos.y, z[2] = f.pos.z;
	if ( m == 12 )
	{
		double R[3][3];
		Rot( vrmath::Normalize( f.rot ), R );
		for ( int c = 0; c < 3; c++ )
			for ( int r = 0; r < 3; r++ )
				z[3 + 3 * c + r] = kTip * R[r][c];
	}
	double y[kMaxM];
	for ( int i = 0; i < m; i++ )
		y[i] = z[i] - ybar[i];

	// K = C S^-1: solve S k_i = c_i for each state row.
	double Ls[kMaxM][kMaxM];
	for ( int i = 0; i < m; i++ )
		for ( int j = 0; j < m; j++ )
			Ls[i][j] = S[i][j];
	if ( !Cholesky( &Ls[0][0], m, kMaxM ) )
		return false;
	double K[N][kMaxM];
	for ( int i = 0; i < N; i++ )
	{
		for ( int j = 0; j < m; j++ )
			K[i][j] = C[i][j];
		CholSolve( &Ls[0][0], m, kMaxM, K[i] );
	}
	double dx[N];
	for ( int i = 0; i < N; i++ )
	{
		double v = 0.0;
		for ( int j = 0; j < m; j++ )
			v += K[i][j] * y[j];
		dx[i] = v;
	}
	// P -= K C^T, symmetrised.
	for ( int i = 0; i < N; i++ )
		for ( int j = 0; j < N; j++ )
		{
			double v = 0.0;
			for ( int k = 0; k < m; k++ )
				v += K[i][k] * C[j][k];
			P_[i][j] -= v;
		}
	for ( int i = 0; i < N; i++ )
		for ( int j = i + 1; j < N; j++ )
			P_[i][j] = P_[j][i] = 0.5 * ( P_[i][j] + P_[j][i] );

	for ( int i = 0; i < N; i++ )
		if ( !std::isfinite( dx[i] ) || !( P_[i][i] > 0.0 ) )
			return false;
	s.p = vrmath::Add( s.p, FromArr( dx + kP ) );
	s.v = vrmath::Add( s.v, FromArr( dx + kV ) );
	s.q = vrmath::Normalize( vrmath::Mul( s.q, Exp( FromArr( dx + kTh ) ) ) );
	s.bg = vrmath::Add( s.bg, FromArr( dx + kBg ) );
	s.ba = vrmath::Add( s.ba, FromArr( dx + kBa ) );
	return true;
}

void ControllerUkf::Guard( State &s )
{
	if ( Norm( s.v ) > kVelLimit )
	{
		s.v = {};
		ResetBlock( kV, kP0Vel );
	}
	if ( std::fabs( s.bg.x ) > kGyroBiasLimit || std::fabs( s.bg.y ) > kGyroBiasLimit ||
	     std::fabs( s.bg.z ) > kGyroBiasLimit )
	{
		s.bg = bg_hint_;
		ResetBlock( kBg, kP0Bg );
	}
	if ( std::fabs( s.ba.x ) > kAccBiasLimit || std::fabs( s.ba.y ) > kAccBiasLimit ||
	     std::fabs( s.ba.z ) > kAccBiasLimit )
	{
		s.ba = {};
		ResetBlock( kBa, kP0Ba );
	}
}

ControllerUkf::Result ControllerUkf::AddFix( const Fix &f_in )
{
	if ( ring_.empty() )
		return kRejected;
	Fix f = f_in;
	if ( f.us > ring_.back().us )
		f.us = ring_.back().us;  // the IMU has not caught up: apply at its newest time
	if ( f.us < ring_.front().us || ( init_ && f.us <= last_fix_us_ ) )
		return kRejected;

	// Last sample at or before the fix time.
	size_t k = ring_.size() - 1;
	while ( k > 0 && ring_[k].us > f.us )
		k--;
	State s = ring_[k].after;
	if ( init_ && k + 1 < ring_.size() && f.us > ring_[k].us )
		Propagate( s, ring_[k + 1].gyro, ring_[k + 1].accel, double( f.us - ring_[k].us ) * 1e-6 );
	s.us = f.us;

	Result res;
	if ( !init_ )
	{
		if ( !f.has_rot )
			return kRejected;
		Init( f, s );
		s = cur_;
		res = kInitialized;
	}
	else
	{
		CovarianceStep( f.us, s );
		const double d = Norm( Sub( f.pos, s.p ) );
		// Sony's gate, widened (own) to 3 sigma of the predicted position so
		// that fixes after an optical dropout, when the IMU has drifted, are
		// taken instead of rejected.
		const double sigma = std::sqrt( std::max( { P_[kP][kP], P_[kP + 1][kP + 1], P_[kP + 2][kP + 2] } ) );
		const double gate = std::max( Norm( s.v ) < kGateSpeed ? kGateNear : kGateFar, kGateSigmas * sigma );
		if ( d > gate )
		{
			// After a stale gap a far fix restarts the filter at once (own).
			const bool stale = f.us > last_fix_us_ + kStaleUs;
			if ( !stale && ++rejects_ < kMaxRejects )
				return kGated;
			Init( f, s );  // lost: start over from the measurement
			s = cur_;
			res = kInitialized;
		}
		else
		{
			if ( !Update( s, f ) )
			{
				Reset();
				return kRejected;
			}
			Guard( s );
			res = kApplied;
		}
	}
	rejects_ = 0;
	last_fix_us_ = f.us;
	fix_pos_ = s.p;

	// Replay the IMU after the fix time.
	cur_ = s;
	if ( ring_[k].us == f.us )
		ring_[k].after = s;
	uint64_t t = f.us;
	for ( size_t j = k + 1; j < ring_.size(); j++ )
	{
		if ( ring_[j].us > t )
		{
			Propagate( cur_, ring_[j].gyro, ring_[j].accel, double( ring_[j].us - t ) * 1e-6 );
			cur_.us = ring_[j].us;
			t = ring_[j].us;
		}
		ring_[j].after = cur_;
	}
	return res;
}
