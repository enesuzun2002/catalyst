#include <stdafx.hpp>

namespace systems {

	namespace detail {

		static constexpr auto k_inner_node_size{ 32ull };
		static constexpr auto k_outer_node_size{ 48ull };

		static constexpr auto k_min_mesh_density{ 1.0e-6f };
		static constexpr auto k_clip_shape_54{ 2u };

		struct inner_node_t
		{
			float min[ 3 ];
			std::uint32_t packed0;
			float max[ 3 ];
			std::uint32_t packed1;

			[[nodiscard]] std::uint32_t type( ) const { return packed0 >> 30; }
			[[nodiscard]] std::uint32_t payload( ) const { return packed0 & 0x3fffffffu; }
		};

		struct hedge_t { std::uint8_t next, twin, vert, face; };
		struct quat_t { float x, y, z, w; };
		struct mat3_t { float m[ 3 ][ 3 ]; };

		static mat3_t quat_to_matrix( const quat_t& q )
		{
			const auto xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
			const auto xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
			const auto wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

			return
			{ {
				{ 1 - 2 * ( yy + zz ), 2 * ( xy + wz ),     2 * ( xz - wy )     },
				{ 2 * ( xy - wz ),     1 - 2 * ( xx + zz ), 2 * ( yz + wx )     },
				{ 2 * ( xz + wy ),     2 * ( yz - wx ),     1 - 2 * ( xx + yy ) },
			} };
		}

		static math::vector3 transform_point( const mat3_t& rot, const float scale[ 3 ], const float pos[ 3 ], const math::vector3& local )
		{
			const auto sx = local.x * scale[ 0 ], sy = local.y * scale[ 1 ], sz = local.z * scale[ 2 ];

			return
			{
				rot.m[ 0 ][ 0 ] * sx + rot.m[ 1 ][ 0 ] * sy + rot.m[ 2 ][ 0 ] * sz + pos[ 0 ],
				rot.m[ 0 ][ 1 ] * sx + rot.m[ 1 ][ 1 ] * sy + rot.m[ 2 ][ 1 ] * sz + pos[ 1 ],
				rot.m[ 0 ][ 2 ] * sx + rot.m[ 1 ][ 2 ] * sy + rot.m[ 2 ][ 2 ] * sz + pos[ 2 ],
			};
		}

		static bool extract_mesh( std::uintptr_t bvh_ptr, std::uintptr_t vert_ptr, std::uintptr_t tri_ptr, std::uint32_t node_count, const mat3_t& rot, const float scale[ 3 ], const float pos[ 3 ], std::uintptr_t mat_arr_ptr, int mat_count, const std::vector<bvh::global_surface_entry>& global_table, const bvh::surface_info& default_surface, std::vector<bvh::triangle>& out )
		{
			if ( node_count == 0 || node_count > 0x1000000 )
			{
				return false;
			}

			std::vector<std::uint8_t> bvh_buf( static_cast< std::size_t >( node_count ) * k_inner_node_size );
			g::memory.read( bvh_ptr, bvh_buf.data( ), bvh_buf.size( ) );

			std::uint32_t min_tri = UINT32_MAX, max_tri = 0;
			std::vector<std::pair<std::uint32_t, std::uint32_t>> ranges;
			std::vector<std::uint32_t> stack;
			stack.reserve( 256 );

			std::uint32_t cursor{ 0 };

			while ( true )
			{
				if ( cursor >= node_count )
				{
					if ( stack.empty( ) )
					{
						break;
					}

					cursor = stack.back( );
					stack.pop_back( );
					continue;
				}

				const auto node = reinterpret_cast< const inner_node_t* >( bvh_buf.data( ) + static_cast< std::size_t >( cursor ) * k_inner_node_size );
				const auto type = node->type( );
				const auto payload = node->payload( );

				if ( type == 3 )
				{
					if ( payload > 0 && payload < 0x1000000 )
					{
						ranges.push_back( { node->packed1, payload } );
						min_tri = std::min( min_tri, node->packed1 );
						max_tri = std::max( max_tri, node->packed1 + payload );
					}

					if ( stack.empty( ) )
					{
						break;
					}

					cursor = stack.back( );
					stack.pop_back( );
				}
				else if ( payload == 0 )
				{
					if ( stack.empty( ) )
					{
						break;
					}

					cursor = stack.back( );
					stack.pop_back( );
				}
				else
				{
					if ( cursor + payload < node_count )
					{
						stack.push_back( cursor + payload );
					}

					++cursor;
				}
			}

			if ( ranges.empty( ) || max_tri <= min_tri )
			{
				return false;
			}

			const auto total_tris = max_tri - min_tri;
			if ( total_tris > 0x1000000 )
			{
				return false;
			}

			{
				const auto root = reinterpret_cast< const inner_node_t* >( bvh_buf.data( ) );
				const auto volume = static_cast< double >( root->max[ 0 ] - root->min[ 0 ] ) * static_cast< double >( root->max[ 1 ] - root->min[ 1 ] ) * static_cast< double >( root->max[ 2 ] - root->min[ 2 ] );

				if ( volume > 0.0 && static_cast< float >( total_tris / volume ) < k_min_mesh_density )
				{
					return false;
				}
			}

			std::vector<int> indices( total_tris * 3 );
			g::memory.read( tri_ptr + static_cast< std::uintptr_t >( min_tri ) * 12, indices.data( ), total_tris * 12 );

			auto max_vert{ 0 };

			for ( const auto idx : indices )
			{
				max_vert = std::max( max_vert, idx );
			}

			if ( max_vert <= 0 || max_vert > 0x1000000 )
			{
				return false;
			}

			const auto vert_count = static_cast< std::uint32_t >( max_vert + 1 );
			const bool has_materials = mat_arr_ptr > 0x10000 && mat_count > 0;

			std::vector<float> vertices( vert_count * 3 );
			g::memory.read( vert_ptr, vertices.data( ), static_cast< std::size_t >( vert_count ) * 12 );

			std::vector<std::uint8_t> materials;

			if ( has_materials )
			{
				materials.resize( total_tris );
				g::memory.read( mat_arr_ptr + static_cast< std::uintptr_t >( min_tri ), materials.data( ), total_tris );
			}

			const auto global_count = static_cast< int >( global_table.size( ) );
			const auto before = out.size( );

			for ( const auto& [start, count] : ranges )
			{
				for ( auto i = 0u; i < count; ++i )
				{
					const auto local_idx = start - min_tri + i;
					if ( local_idx >= total_tris )
					{
						continue;
					}

					auto surf = default_surface;
					if ( has_materials )
					{
						const auto gi = materials[ local_idx ];
						if ( gi < global_count )
						{
							const auto& gs = global_table[ gi ];
							surf.penetration = gs.penetration_mod;
							surf.surface_type = gs.surface_type;
							surf.global_index = gi;
						}
					}

					const auto base = local_idx * 3;
					const auto i0 = indices[ base ];
					const auto i1 = indices[ static_cast< std::size_t >( base ) + 1 ];
					const auto i2 = indices[ static_cast< std::size_t >( base ) + 2 ];

					if ( static_cast< std::uint32_t >( i0 ) >= vert_count || static_cast< std::uint32_t >( i1 ) >= vert_count || static_cast< std::uint32_t >( i2 ) >= vert_count )
					{
						continue;
					}

					const auto xf = [ & ]( int vi ) { return transform_point( rot, scale, pos, { vertices[ vi * 3 ], vertices[ vi * 3 + 1 ], vertices[ vi * 3 + 2 ] } ); };
					out.push_back( { .v0 = xf( i0 ), .v1 = xf( i1 ), .v2 = xf( i2 ), .surface = surf } );
				}
			}

			return out.size( ) > before;
		}

		static bool extract_hull( std::uintptr_t hull_data, float uniform_scale, const bvh::surface_info& surface, std::vector<bvh::triangle>& out )
		{
			if ( !hull_data )
			{
				return false;
			}

			std::uint8_t hd[ 0x100 ]{};
			g::memory.read( hull_data, hd, sizeof( hd ) );

			const auto vert_count = *reinterpret_cast< const int* >( hd + 0x88 );
			const auto vert_ptr = *reinterpret_cast< const std::uintptr_t* >( hd + 0x90 );
			const auto hedge_count = *reinterpret_cast< const int* >( hd + 0xa0 );
			const auto hedge_ptr = *reinterpret_cast< const std::uintptr_t* >( hd + 0xa8 );
			const auto face_count = *reinterpret_cast< const int* >( hd + 0xb8 );
			const auto face_ptr = *reinterpret_cast< const std::uintptr_t* >( hd + 0xc0 );

			if ( vert_count <= 0 || vert_count > 0xffff || hedge_count <= 0 || hedge_count > 0xffff || face_count <= 0 || face_count > 0xffff || !vert_ptr || !hedge_ptr || !face_ptr )
			{
				return false;
			}

			std::vector<float> verts( vert_count * 3 );
			std::vector<hedge_t> hedges( hedge_count );
			std::vector<std::uint8_t> faces( face_count );

			g::memory.read( vert_ptr, verts.data( ), static_cast< std::size_t >( vert_count ) * 12 );
			g::memory.read( hedge_ptr, hedges.data( ), static_cast< std::size_t >( hedge_count ) * 4 );
			g::memory.read( face_ptr, faces.data( ), face_count );

			const auto before = out.size( );
			const auto vert = [ & ]( int vi ) -> math::vector3
				{
					return
					{
						verts[ vi * 3 ] * uniform_scale,
						verts[ vi * 3 + 1 ] * uniform_scale,
						verts[ vi * 3 + 2 ] * uniform_scale,
					};
				};

			std::vector<int> face_verts;
			face_verts.reserve( 16 );

			for ( auto fi = 0; fi < face_count; ++fi )
			{
				const auto start_he = faces[ fi ];
				if ( start_he >= hedge_count )
				{
					continue;
				}

				face_verts.clear( );

				auto he = start_he;
				auto safety{ 0 };

				do
				{
					face_verts.push_back( hedges[ he ].vert );
					he = hedges[ he ].next;
				} while ( he != start_he && ++safety < 64 && he < hedge_count );

				if ( face_verts.size( ) < 3 )
				{
					continue;
				}

				const auto v0 = vert( face_verts[ 0 ] );

				for ( auto i = 1ull; i + 1 < face_verts.size( ); ++i )
				{
					out.push_back( { .v0 = v0, .v1 = vert( face_verts[ i ] ), .v2 = vert( face_verts[ i + 1 ] ), .surface = surface } );
				}
			}

			return out.size( ) > before;
		}

		static void process_shape( std::uintptr_t shape_body, std::uintptr_t hull_vtable, std::uintptr_t mesh_vtable, const std::vector<bvh::global_surface_entry>& global_table, std::vector<bvh::triangle>& out )
		{
			const auto vtable = g::memory.read<std::uintptr_t>( shape_body );

			if ( vtable == hull_vtable )
			{
				const auto hull_data = g::memory.read<std::uintptr_t>( shape_body + 0xb8 );
				if ( !hull_data )
				{
					return;
				}

				const auto scale = g::memory.read<float>( shape_body + 0xb0 );
				const auto surf = bvh::surface_info{ .penetration = g::memory.read<float>( shape_body + 0x28 ), .kind = bvh::shape_kind::hull };

				extract_hull( hull_data, ( scale > 0.0f && std::isfinite( scale ) ) ? scale : 1.0f, surf, out );

				return;
			}

			if ( vtable != mesh_vtable )
			{
				return;
			}

			const auto mesh_data = g::memory.read<std::uintptr_t>( shape_body + 0xc0 );
			if ( !mesh_data )
			{
				return;
			}

			std::uint8_t md[ 0xA0 ]{};
			g::memory.read( mesh_data, md, sizeof( md ) );

			const auto mat_count = *reinterpret_cast< const int* >( md + 0x90 );
			const auto mat_arr_ptr = *reinterpret_cast< const std::uintptr_t* >( md + 0x98 );

			if ( mat_count <= 0 )
			{
				return;
			}

			const auto default_damage = g::memory.read<float>( shape_body + 0x2c );
			if ( default_damage < 0.0f )
			{
				return;
			}

			float scale[ 3 ]{};
			g::memory.read( shape_body + 0xB0, scale, 12 );

			if ( !std::isfinite( scale[ 0 ] ) || !std::isfinite( scale[ 1 ] ) || !std::isfinite( scale[ 2 ] ) || ( scale[ 0 ] == 0.0f && scale[ 1 ] == 0.0f && scale[ 2 ] == 0.0f ) )
			{
				return;
			}

			float world_pos[ 3 ]{};
			g::memory.read( shape_body + 0x100, world_pos, 12 );

			quat_t quat{};
			g::memory.read( shape_body + 0x130, &quat, sizeof( quat ) );

			const auto ql = quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
			if ( ql < 0.5f || ql > 1.5f )
			{
				quat = { 0, 0, 0, 1 };
			}

			auto node_count = *reinterpret_cast< const std::uint32_t* >( md + 0x18 );
			if ( node_count == 0 || node_count > 0x1000000 )
			{
				node_count = *reinterpret_cast< const std::uint32_t* >( md + 0x28 ) & 0x3FFFFFFFu;
				if ( node_count == 0 || node_count > 0x1000000 )
				{
					return;
				}
			}

			bvh::surface_info default_surface{ .penetration = g::memory.read<float>( shape_body + 0x28 ), .kind = bvh::shape_kind::mesh };
			extract_mesh( *reinterpret_cast< const std::uintptr_t* >( md + 0x20 ), *reinterpret_cast< const std::uintptr_t* >( md + 0x38 ), *reinterpret_cast< const std::uintptr_t* >( md + 0x50 ), node_count, quat_to_matrix( quat ), scale, world_pos, mat_arr_ptr, mat_count, global_table, default_surface, out );
		}

	} // namespace detail

	void bvh::parse( )
	{
		const auto trace_call = g::memory.find_pattern( g::modules.client, "E8 ? ? ? ? C7 87 ? ? ? ? ? ? ? ? 48 8D 54 24 ? 48 8B CF" );
		const auto vphys2_world = g::memory.read<std::uintptr_t>( g::memory.read<std::uintptr_t>( g::memory.resolve_rip( trace_call - 0x1a ) ) );

		if ( !vphys2_world )
		{
			return;
		}

		const auto surface_fn = g::memory.find_pattern( g::modules.client, "48 63 41 ? 48 8B 0D" );
		const auto surface_manager = g::memory.read<std::uintptr_t>( g::memory.resolve_rip( surface_fn + 4 ) );

		if ( !surface_manager )
		{
			return;
		}

		std::vector<global_surface_entry> global_table;

		const auto array_base = g::memory.read<std::uintptr_t>( surface_manager + 40 );
		if ( array_base )
		{
			auto surface_count{ 0 };

			for ( const auto off : { 32, 36, 24, 28, 48 } )
			{
				if ( const auto c = g::memory.read<int>( surface_manager + off ); c > 0 )
				{
					surface_count = c;
					break;
				}
			}

			if ( surface_count <= 0 )
			{
				for ( auto i = 0; i < 1024; ++i )
				{
					global_surface_entry sd{};
					g::memory.read( array_base + static_cast< std::size_t >( i ) * 32, &sd, sizeof( sd ) );

					if ( sd.penetration_mod == 0.0f && sd.surface_type == 0 && sd.unk_00 == 0.0f )
					{
						if ( surface_count > 0 && i - surface_count > 8 )
						{
							break;
						}

						continue;
					}

					surface_count = i + 1;
				}
			}
			if ( surface_count > 0 )
			{
				global_table.resize( surface_count );
				g::memory.read( array_base, global_table.data( ), static_cast< std::size_t >( surface_count ) * sizeof( global_surface_entry ) );
			}
		}

		const auto inner_world = g::memory.read<std::uintptr_t>( vphys2_world + 0x30 );
		const auto body_array = inner_world ? g::memory.read<std::uintptr_t>( inner_world + 0x110 ) : 0;
		const auto body_count = body_array ? g::memory.read<int>( body_array + 0x268 ) : 0;

		if ( !body_count )
		{
			return;
		}

		const auto hull_vtable = g::memory.find_vtable( g::modules.vphysics2, "CRnHullShape" );
		const auto mesh_vtable = g::memory.find_vtable( g::modules.vphysics2, "CRnMeshShape" );

		if ( !hull_vtable || !mesh_vtable )
		{
			return;
		}

		std::vector<triangle> fresh;
		fresh.reserve( 262144 );

		const auto try_shape = [ & ]( std::uintptr_t shape )
			{
				if ( !shape )
				{
					return;
				}

				if ( g::memory.read<std::uint32_t>( shape + 0x54 ) == detail::k_clip_shape_54 )
				{
					return;
				}

				detail::process_shape( shape, hull_vtable, mesh_vtable, global_table, fresh );
			};

		for ( auto body_idx = 0; body_idx < body_count; ++body_idx )
		{
			const auto body = body_array + static_cast< std::uintptr_t >( body_idx ) * 88;
			const auto bvh_nodes_ptr = g::memory.read<std::uintptr_t>( body + 0x18 );

			if ( !bvh_nodes_ptr )
			{
				continue;
			}

			if ( g::memory.read<std::uint32_t>( body + 0x40 ) != 2 )
			{
				continue;
			}

			const auto bvh_root = g::memory.read<int>( body );
			if ( bvh_root < 0 )
			{
				try_shape( g::memory.read<std::uintptr_t>( body + 0x28 ) );
				continue;
			}

			const auto outer_node_count = std::max( { static_cast< std::uint32_t >( bvh_root + 1 ), static_cast< std::uint32_t >( g::memory.read<int>( body + 0x08 ) ), static_cast< std::uint32_t >( g::memory.read<int>( body + 0x10 ) ), } );
			if ( outer_node_count > 0x100000 )
			{
				continue;
			}

			std::vector<std::uint8_t> outer_buf( outer_node_count * detail::k_outer_node_size );
			g::memory.read( bvh_nodes_ptr, outer_buf.data( ), outer_buf.size( ) );

			std::vector<int> outer_stack{ bvh_root };
			outer_stack.reserve( 128 );
			std::unordered_set<std::uintptr_t> seen;

			while ( !outer_stack.empty( ) )
			{
				const auto idx = outer_stack.back( );
				outer_stack.pop_back( );
				if ( idx < 0 || static_cast< std::uint32_t >( idx ) >= outer_node_count )
				{
					continue;
				}

				const auto node = outer_buf.data( ) + static_cast< std::uintptr_t >( idx ) * detail::k_outer_node_size;
				const auto left = *reinterpret_cast< const int* >( node + 12 );

				if ( left == -1 )
				{
					const auto shape = *reinterpret_cast< const std::uintptr_t* >( node + 0x28 );
					if ( shape && seen.insert( shape ).second )
					{
						try_shape( shape );
					}

					continue;
				}

				const auto right = *reinterpret_cast< const int* >( node + 28 );

				if ( left >= 0 )
				{
					outer_stack.push_back( left );
				}

				if ( right >= 0 )
				{
					outer_stack.push_back( right );
				}
			}
		}

		{
			std::unique_lock lock( this->m_mutex );
			this->m_triangles = std::move( fresh );
		}

		this->rebuild_accel( );
	}

	void bvh::clear( )
	{
		std::unique_lock lock( this->m_mutex );
		this->m_triangles.clear( );
		this->m_nodes.clear( );
		this->m_indices.clear( );
		this->m_tri_bounds.clear( );
		this->m_centroids.clear( );
	}

	bvh::trace_result bvh::trace_ray( const math::vector3& start, const math::vector3& end, int exclude_tri ) const
	{
		trace_result result{ .end_pos = end };
		if ( this->m_nodes.empty( ) )
		{
			return result;
		}

		const auto dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
		const auto max_dist = std::sqrt( dx * dx + dy * dy + dz * dz );

		if ( max_dist < 1e-8f )
		{
			return result;
		}

		const auto inv_dist = 1.0f / max_dist;

		const float dir[ 3 ]{ dx * inv_dist, dy * inv_dist, dz * inv_dist };
		const float origin[ 3 ]{ start.x, start.y, start.z };
		const float inv_dir[ 3 ]
		{
			std::abs( dir[ 0 ] ) > 1e-8f ? 1.0f / dir[ 0 ] : ( dir[ 0 ] >= 0 ? 1e12f : -1e12f ),
			std::abs( dir[ 1 ] ) > 1e-8f ? 1.0f / dir[ 1 ] : ( dir[ 1 ] >= 0 ? 1e12f : -1e12f ),
			std::abs( dir[ 2 ] ) > 1e-8f ? 1.0f / dir[ 2 ] : ( dir[ 2 ] >= 0 ? 1e12f : -1e12f ),
		};

		auto closest_t = max_dist;
		auto sp{ 0 };
		int stack[ 128 ]{ 0 };

		while ( sp >= 0 )
		{
			const auto& node = this->m_nodes[ stack[ sp-- ] ];
			if ( !node.bounds.intersects_ray( origin, inv_dir, closest_t ) )
			{
				continue;
			}

			if ( node.left != -1 )
			{
				if ( sp + 2 < 127 )
				{
					stack[ ++sp ] = node.right;
					stack[ ++sp ] = node.left;
				}

				continue;
			}

			for ( auto i = node.tri_start; i < node.tri_start + node.tri_count; ++i )
			{
				const auto ti = this->m_indices[ i ];
				if ( ti == exclude_tri )
				{
					continue;
				}

				const auto& tri = this->m_triangles[ ti ];
				const auto e1x = tri.v1.x - tri.v0.x, e1y = tri.v1.y - tri.v0.y, e1z = tri.v1.z - tri.v0.z;
				const auto e2x = tri.v2.x - tri.v0.x, e2y = tri.v2.y - tri.v0.y, e2z = tri.v2.z - tri.v0.z;
				const auto hx = dir[ 1 ] * e2z - dir[ 2 ] * e2y;
				const auto hy = dir[ 2 ] * e2x - dir[ 0 ] * e2z;
				const auto hz = dir[ 0 ] * e2y - dir[ 1 ] * e2x;
				const auto a = e1x * hx + e1y * hy + e1z * hz;

				if ( std::abs( a ) < 1e-8f )
				{
					continue;
				}

				const auto f = 1.0f / a;
				const auto sx = origin[ 0 ] - tri.v0.x, sy = origin[ 1 ] - tri.v0.y, sz = origin[ 2 ] - tri.v0.z;
				const auto u = f * ( sx * hx + sy * hy + sz * hz );

				if ( u < 0.0f || u > 1.0f )
				{
					continue;
				}

				const auto qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
				const auto v = f * ( dir[ 0 ] * qx + dir[ 1 ] * qy + dir[ 2 ] * qz );

				if ( v < 0.0f || u + v > 1.0f )
				{
					continue;
				}

				const auto t = f * ( e2x * qx + e2y * qy + e2z * qz );
				if ( t <= 1e-5f || t >= closest_t )
				{
					continue;
				}

				closest_t = t;
				result.hit = true;
				result.fraction = t / max_dist;
				result.distance = t;
				result.triangle_index = ti;
				result.surface = tri.surface;
				result.end_pos = { origin[ 0 ] + dir[ 0 ] * t, origin[ 1 ] + dir[ 1 ] * t, origin[ 2 ] + dir[ 2 ] * t };

				const auto nx = e1y * e2z - e1z * e2y;
				const auto ny = e1z * e2x - e1x * e2z;
				const auto nz = e1x * e2y - e1y * e2x;
				const auto nl = std::sqrt( nx * nx + ny * ny + nz * nz );

				if ( nl > 1e-8f )
				{
					const auto inv_nl = 1.0f / nl;
					result.normal = { nx * inv_nl, ny * inv_nl, nz * inv_nl };
				}
			}
		}

		return result;
	}

	bvh::trace_result bvh::trace_hull( const math::vector3& start, const math::vector3& end, const math::vector3& hull_mins, const math::vector3& hull_maxs, int exclude_tri ) const
	{
		const float half[ 3 ]
		{
			( hull_maxs.x - hull_mins.x ) * 0.5f,
			( hull_maxs.y - hull_mins.y ) * 0.5f,
			( hull_maxs.z - hull_mins.z ) * 0.5f,
		};

		const float offset[ 3 ]
		{
			( hull_mins.x + hull_maxs.x ) * 0.5f,
			( hull_mins.y + hull_maxs.y ) * 0.5f,
			( hull_mins.z + hull_maxs.z ) * 0.5f,
		};

		trace_result result{ .end_pos = end };
		if ( this->m_nodes.empty( ) )
		{
			return result;
		}

		const auto dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
		const auto max_dist = std::sqrt( dx * dx + dy * dy + dz * dz );

		if ( max_dist < 1e-8f )
		{
			return result;
		}

		const auto inv_dist = 1.0f / max_dist;

		const float dir[ 3 ]{ dx * inv_dist, dy * inv_dist, dz * inv_dist };
		const float origin[ 3 ]{ start.x + offset[ 0 ], start.y + offset[ 1 ], start.z + offset[ 2 ] };
		const float inv_dir[ 3 ]
		{
			std::abs( dir[ 0 ] ) > 1e-8f ? 1.0f / dir[ 0 ] : ( dir[ 0 ] >= 0 ? 1e12f : -1e12f ),
			std::abs( dir[ 1 ] ) > 1e-8f ? 1.0f / dir[ 1 ] : ( dir[ 1 ] >= 0 ? 1e12f : -1e12f ),
			std::abs( dir[ 2 ] ) > 1e-8f ? 1.0f / dir[ 2 ] : ( dir[ 2 ] >= 0 ? 1e12f : -1e12f ),
		};

		auto closest_t = max_dist;
		auto sp{ 0 };
		int stack[ 128 ]{ 0 };

		while ( sp >= 0 )
		{
			const auto& node = this->m_nodes[ stack[ sp-- ] ];
			auto expanded = node.bounds;

			for ( auto i = 0; i < 3; ++i )
			{
				expanded.mins[ i ] -= half[ i ];
				expanded.maxs[ i ] += half[ i ];
			}

			if ( !expanded.intersects_ray( origin, inv_dir, closest_t ) )
			{
				continue;
			}

			if ( node.left != -1 )
			{
				if ( sp + 2 < 127 )
				{
					stack[ ++sp ] = node.right;
					stack[ ++sp ] = node.left;
				}

				continue;
			}

			for ( auto i = node.tri_start; i < node.tri_start + node.tri_count; ++i )
			{
				const auto ti = this->m_indices[ i ];
				if ( ti == exclude_tri )
				{
					continue;
				}

				const auto& tri = this->m_triangles[ ti ];
				const auto e1x = tri.v1.x - tri.v0.x, e1y = tri.v1.y - tri.v0.y, e1z = tri.v1.z - tri.v0.z;
				const auto e2x = tri.v2.x - tri.v0.x, e2y = tri.v2.y - tri.v0.y, e2z = tri.v2.z - tri.v0.z;

				auto nx = e1y * e2z - e1z * e2y;
				auto ny = e1z * e2x - e1x * e2z;
				auto nz = e1x * e2y - e1y * e2x;

				const auto nl = std::sqrt( nx * nx + ny * ny + nz * nz );
				if ( nl < 1e-8f )
				{
					continue;
				}

				const auto inv_nl = 1.0f / nl;
				nx *= inv_nl; ny *= inv_nl; nz *= inv_nl;

				const auto support = half[ 0 ] * std::abs( nx ) + half[ 1 ] * std::abs( ny ) + half[ 2 ] * std::abs( nz );
				const auto sign = ( ( origin[ 0 ] - tri.v0.x ) * nx + ( origin[ 1 ] - tri.v0.y ) * ny + ( origin[ 2 ] - tri.v0.z ) * nz ) >= 0.0f ? 1.0f : -1.0f;
				const auto px = nx * support * sign, py = ny * support * sign, pz = nz * support * sign;

				const auto v0x = tri.v0.x + px, v0y = tri.v0.y + py, v0z = tri.v0.z + pz;
				const auto fe1x = tri.v1.x + px - v0x, fe1y = tri.v1.y + py - v0y, fe1z = tri.v1.z + pz - v0z;
				const auto fe2x = tri.v2.x + px - v0x, fe2y = tri.v2.y + py - v0y, fe2z = tri.v2.z + pz - v0z;

				const auto hx = dir[ 1 ] * fe2z - dir[ 2 ] * fe2y;
				const auto hy = dir[ 2 ] * fe2x - dir[ 0 ] * fe2z;
				const auto hz = dir[ 0 ] * fe2y - dir[ 1 ] * fe2x;
				const auto a = fe1x * hx + fe1y * hy + fe1z * hz;

				if ( std::abs( a ) < 1e-8f )
				{
					continue;
				}

				const auto f = 1.0f / a;
				const auto sx = origin[ 0 ] - v0x, sy = origin[ 1 ] - v0y, sz = origin[ 2 ] - v0z;
				const auto u = f * ( sx * hx + sy * hy + sz * hz );

				if ( u < -0.01f || u > 1.01f )
				{
					continue;
				}

				const auto qx = sy * fe1z - sz * fe1y, qy = sz * fe1x - sx * fe1z, qz = sx * fe1y - sy * fe1x;
				const auto v = f * ( dir[ 0 ] * qx + dir[ 1 ] * qy + dir[ 2 ] * qz );

				if ( v < -0.01f || u + v > 1.02f )
				{
					continue;
				}

				const auto t = f * ( fe2x * qx + fe2y * qy + fe2z * qz );
				if ( t <= 0.0f || t >= closest_t )
				{
					continue;
				}

				closest_t = t;
				result.hit = true;
				result.fraction = t / max_dist;
				result.distance = t;
				result.triangle_index = ti;
				result.surface = tri.surface;
				result.normal = { nx * sign, ny * sign, nz * sign };
				result.end_pos = { start.x + dir[ 0 ] * t, start.y + dir[ 1 ] * t, start.z + dir[ 2 ] * t };
			}
		}

		return result;
	}

	std::vector<bvh::hit_entry> bvh::trace_ray_all( const math::vector3& start, const math::vector3& end ) const
	{
		std::vector<hit_entry> hits;
		if ( this->m_nodes.empty( ) )
		{
			return hits;
		}

		const auto dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
		const auto max_dist = std::sqrt( dx * dx + dy * dy + dz * dz );

		if ( max_dist < 1e-8f )
		{
			return hits;
		}

		const auto inv_dist = 1.0f / max_dist;

		const float dir[ 3 ]{ dx * inv_dist, dy * inv_dist, dz * inv_dist };
		const float origin[ 3 ]{ start.x, start.y, start.z };
		const float inv_dir[ 3 ]
		{
			std::abs( dir[ 0 ] ) > 1e-8f ? 1.0f / dir[ 0 ] : ( dir[ 0 ] >= 0 ? 1e12f : -1e12f ),
			std::abs( dir[ 1 ] ) > 1e-8f ? 1.0f / dir[ 1 ] : ( dir[ 1 ] >= 0 ? 1e12f : -1e12f ),
			std::abs( dir[ 2 ] ) > 1e-8f ? 1.0f / dir[ 2 ] : ( dir[ 2 ] >= 0 ? 1e12f : -1e12f ),
		};

		int stack[ 128 ]{ 0 };
		int sp{ 0 };

		while ( sp >= 0 )
		{
			const auto& node = this->m_nodes[ stack[ sp-- ] ];
			if ( !node.bounds.intersects_ray( origin, inv_dir, max_dist ) )
			{
				continue;
			}

			if ( node.left != -1 )
			{
				if ( sp + 2 < 127 )
				{
					stack[ ++sp ] = node.right;
					stack[ ++sp ] = node.left;
				}

				continue;
			}

			for ( int i = node.tri_start; i < node.tri_start + node.tri_count; ++i )
			{
				const auto ti = this->m_indices[ i ];
				const auto& tri = this->m_triangles[ ti ];
				const auto e1x = tri.v1.x - tri.v0.x, e1y = tri.v1.y - tri.v0.y, e1z = tri.v1.z - tri.v0.z;
				const auto e2x = tri.v2.x - tri.v0.x, e2y = tri.v2.y - tri.v0.y, e2z = tri.v2.z - tri.v0.z;
				const auto hx = dir[ 1 ] * e2z - dir[ 2 ] * e2y;
				const auto hy = dir[ 2 ] * e2x - dir[ 0 ] * e2z;
				const auto hz = dir[ 0 ] * e2y - dir[ 1 ] * e2x;
				const auto a = e1x * hx + e1y * hy + e1z * hz;

				if ( std::abs( a ) < 1e-8f )
				{
					continue;
				}

				const auto f = 1.0f / a;
				const auto sx = origin[ 0 ] - tri.v0.x, sy = origin[ 1 ] - tri.v0.y, sz = origin[ 2 ] - tri.v0.z;
				const auto u = f * ( sx * hx + sy * hy + sz * hz );

				if ( u < 0.0f || u > 1.0f )
				{
					continue;
				}

				const auto qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
				const auto v = f * ( dir[ 0 ] * qx + dir[ 1 ] * qy + dir[ 2 ] * qz );

				if ( v < 0.0f || u + v > 1.0f )
				{
					continue;
				}

				const auto t = f * ( e2x * qx + e2y * qy + e2z * qz );
				if ( t <= 1e-5f || t >= max_dist )
				{
					continue;
				}

				auto nx = e1y * e2z - e1z * e2y;
				auto ny = e1z * e2x - e1x * e2z;
				auto nz = e1x * e2y - e1y * e2x;

				if ( const auto nl = std::sqrt( nx * nx + ny * ny + nz * nz ); nl > 1e-8f )
				{
					const auto inv_nl = 1.0f / nl;
					nx *= inv_nl; ny *= inv_nl; nz *= inv_nl;
				}

				hits.push_back(
					{
					.distance = t,
					.fraction = t / max_dist,
					.position = { origin[ 0 ] + dir[ 0 ] * t, origin[ 1 ] + dir[ 1 ] * t, origin[ 2 ] + dir[ 2 ] * t },
					.normal = { nx, ny, nz },
					.surface = tri.surface,
					.triangle_index = ti,
					.is_enter = ( nx * dir[ 0 ] + ny * dir[ 1 ] + nz * dir[ 2 ] ) < 0.0f,
					} );
			}
		}

		std::sort( hits.begin( ), hits.end( ), [ ]( const hit_entry& a, const hit_entry& b ) { return a.distance < b.distance; } );

		if ( hits.size( ) >= 2 )
		{
			std::vector<hit_entry> deduped;
			deduped.reserve( hits.size( ) );
			deduped.push_back( hits.front( ) );

			for ( auto i = 1ull; i < hits.size( ); ++i )
			{
				const auto& prev = deduped.back( );
				const auto& cur = hits[ i ];
				const auto ndot = cur.normal.x * prev.normal.x + cur.normal.y * prev.normal.y + cur.normal.z * prev.normal.z;

				if ( cur.distance - prev.distance < 0.05f && ndot > 0.95f )
				{
					continue;
				}

				deduped.push_back( cur );
			}

			hits = std::move( deduped );
		}

		return hits;
	}

	std::vector<bvh::penetration_segment> bvh::build_segments( const std::vector<hit_entry>& hits, float /*ray_length*/ ) const
	{
		std::vector<penetration_segment> segments;
		if ( hits.empty( ) )
		{
			return segments;
		}

		auto inside{ 0 };
		auto seg_enter_idx{ -1 };
		auto min_pen = std::numeric_limits<float>::infinity( );

		const auto track_pen = [ & ]( float p )
			{
				if ( p > 0.0f && p < min_pen )
				{
					min_pen = p;
				}
			};

		for ( auto i = 0ull; i < hits.size( ); ++i )
		{
			const auto& h = hits[ i ];

			if ( h.is_enter )
			{
				if ( inside == 0 )
				{
					seg_enter_idx = static_cast< int >( i );
					min_pen = std::numeric_limits<float>::infinity( );
				}

				++inside;
				track_pen( h.surface.penetration );

				continue;
			}

			inside = std::max( inside - 1, 0 );
			track_pen( h.surface.penetration );

			if ( inside != 0 || seg_enter_idx < 0 )
			{
				continue;
			}

			const auto& en = hits[ seg_enter_idx ];

			segments.push_back(
				{
				.enter_fraction = en.fraction,
				.exit_fraction = h.fraction,
				.enter_distance = en.distance,
				.exit_distance = h.distance,
				.enter_pos = en.position,
				.exit_pos = h.position,
				.enter_surface = en.surface,
				.exit_surface = h.surface,
				.thickness = std::max( h.distance - en.distance, 1.0f / 256.0f ),
				.min_pen_mod = std::isfinite( min_pen ) ? min_pen : en.surface.penetration,
				} );

			seg_enter_idx = -1;
		}

		if ( seg_enter_idx >= 0 )
		{
			const auto& en = hits[ seg_enter_idx ];
			const auto& last = hits.back( );

			segments.push_back(
				{
				.enter_fraction = en.fraction,
				.exit_fraction = last.fraction,
				.enter_distance = en.distance,
				.exit_distance = last.distance,
				.enter_pos = en.position,
				.exit_pos = last.position,
				.enter_surface = en.surface,
				.exit_surface = last.surface,
				.thickness = std::max( last.distance - en.distance, 1.0f ),
				.min_pen_mod = std::isfinite( min_pen ) ? min_pen : en.surface.penetration,
				} );
		}

		return segments;
	}

	const std::vector<bvh::triangle>& bvh::triangles( ) const { return this->m_triangles; }

	std::size_t bvh::count( ) const
	{
		std::shared_lock lock( this->m_mutex );
		return this->m_triangles.size( );
	}

	bool bvh::valid( ) const
	{
		std::shared_lock lock( this->m_mutex );
		return !this->m_triangles.empty( );
	}

	void bvh::aabb::expand( const math::vector3& p )
	{
		this->mins[ 0 ] = std::min( this->mins[ 0 ], p.x );
		this->mins[ 1 ] = std::min( this->mins[ 1 ], p.y );
		this->mins[ 2 ] = std::min( this->mins[ 2 ], p.z );
		this->maxs[ 0 ] = std::max( this->maxs[ 0 ], p.x );
		this->maxs[ 1 ] = std::max( this->maxs[ 1 ], p.y );
		this->maxs[ 2 ] = std::max( this->maxs[ 2 ], p.z );
	}

	void bvh::aabb::expand( const aabb& o )
	{
		for ( auto i = 0; i < 3; ++i )
		{
			this->mins[ i ] = std::min( this->mins[ i ], o.mins[ i ] );
			this->maxs[ i ] = std::max( this->maxs[ i ], o.maxs[ i ] );
		}
	}

	int bvh::aabb::longest_axis( ) const
	{
		const auto ex = this->maxs[ 0 ] - this->mins[ 0 ];
		const auto ey = this->maxs[ 1 ] - this->mins[ 1 ];
		const auto ez = this->maxs[ 2 ] - this->mins[ 2 ];

		if ( ex >= ey && ex >= ez )
		{
			return 0;
		}

		return ey >= ez ? 1 : 2;
	}

	bool bvh::aabb::intersects_ray( const float origin[ 3 ], const float inv_dir[ 3 ], float max_t ) const
	{
		auto tmin{ 0.0f };
		auto tmax = max_t;

		for ( auto i = 0; i < 3; ++i )
		{
			auto t0 = ( this->mins[ i ] - origin[ i ] ) * inv_dir[ i ];
			auto t1 = ( this->maxs[ i ] - origin[ i ] ) * inv_dir[ i ];

			if ( inv_dir[ i ] < 0.0f )
			{
				std::swap( t0, t1 );
			}

			tmin = std::max( tmin, t0 );
			tmax = std::min( tmax, t1 );

			if ( tmax < tmin )
			{
				return false;
			}
		}

		return true;
	}

	void bvh::rebuild_accel( )
	{
		this->m_nodes.clear( );
		this->m_indices.clear( );
		this->m_tri_bounds.clear( );
		this->m_centroids.clear( );

		const auto tri_count = static_cast< int >( this->m_triangles.size( ) );
		if ( tri_count == 0 )
		{
			return;
		}

		this->m_indices.resize( tri_count );
		this->m_tri_bounds.resize( tri_count );
		this->m_centroids.resize( static_cast< std::size_t >( tri_count ) * 3 );

		for ( auto i = 0; i < tri_count; ++i )
		{
			this->m_indices[ i ] = i;

			aabb bb{};
			bb.expand( this->m_triangles[ i ].v0 );
			bb.expand( this->m_triangles[ i ].v1 );
			bb.expand( this->m_triangles[ i ].v2 );

			this->m_tri_bounds[ i ] = bb;

			const auto ci = static_cast< std::size_t >( i ) * 3;
			this->m_centroids[ ci ] = ( bb.mins[ 0 ] + bb.maxs[ 0 ] ) * 0.5f;
			this->m_centroids[ ci + 1 ] = ( bb.mins[ 1 ] + bb.maxs[ 1 ] ) * 0.5f;
			this->m_centroids[ ci + 2 ] = ( bb.mins[ 2 ] + bb.maxs[ 2 ] ) * 0.5f;
		}

		this->m_nodes.reserve( static_cast< std::size_t >( tri_count ) * 2 );
		this->build_recursive( 0, tri_count, 0 );
	}

	int bvh::build_recursive( int start, int end, int depth )
	{
		const auto node_idx = static_cast< int >( this->m_nodes.size( ) );
		this->m_nodes.push_back( {} );

		auto& node = this->m_nodes[ node_idx ];
		const auto count = end - start;

		for ( auto i = start; i < end; ++i )
		{
			node.bounds.expand( this->m_tri_bounds[ this->m_indices[ i ] ] );
		}

		if ( count <= k_max_leaf_tris || depth >= k_max_depth )
		{
			node.tri_start = start;
			node.tri_count = count;
			return node_idx;
		}

		aabb centroid_bounds{};

		for ( auto i = start; i < end; ++i )
		{
			const auto ci = static_cast< std::size_t >( this->m_indices[ i ] ) * 3;
			centroid_bounds.expand( math::vector3{ this->m_centroids[ ci ], this->m_centroids[ ci + 1 ], this->m_centroids[ ci + 2 ] } );
		}

		const auto axis = centroid_bounds.longest_axis( );
		const auto mid = ( centroid_bounds.mins[ axis ] + centroid_bounds.maxs[ axis ] ) * 0.5f;

		auto pivot = std::partition( this->m_indices.begin( ) + start, this->m_indices.begin( ) + end, [ & ]( int idx ) { return this->m_centroids[ static_cast< std::size_t >( idx ) * 3 + axis ] < mid; } );
		auto split = static_cast< int >( pivot - this->m_indices.begin( ) );

		if ( split == start || split == end )
		{
			split = start + count / 2;
		}

		node.left = this->build_recursive( start, split, depth + 1 );
		this->m_nodes[ node_idx ].right = this->build_recursive( split, end, depth + 1 );
		return node_idx;
	}

} // namespace systems
