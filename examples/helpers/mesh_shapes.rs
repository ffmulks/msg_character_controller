//! Helper functions for creating 2D meshes that match collider shapes.
//!
//! These functions create visual meshes that accurately represent the shape
//! of physics colliders (capsules, circles, triangles, rectangles).

use bevy::prelude::*;
use bevy::render::mesh::Indices;
use bevy::render::render_resource::PrimitiveTopology;
use std::f32::consts::PI;

/// Creates a 2D capsule mesh (stadium shape).
///
/// A capsule is a rectangle with semicircular caps on top and bottom.
///
/// # Arguments
/// * `half_height` - Half the height of the rectangular portion
/// * `radius` - Radius of the semicircular caps
/// * `segments` - Number of segments for each semicircle (more = smoother)
pub fn create_capsule_mesh(half_height: f32, radius: f32, segments: usize) -> Mesh {
    let segments = segments.max(4);

    // Total vertices: segments for top semicircle + segments for bottom semicircle + 2 for center
    let vertex_count = segments * 2 + 2;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(vertex_count);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(vertex_count);
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(vertex_count);

    // Center point for triangle fan
    positions.push([0.0, 0.0, 0.0]);
    normals.push([0.0, 0.0, 1.0]);
    uvs.push([0.5, 0.5]);

    // Top semicircle (from left to right)
    for i in 0..=segments {
        let angle = PI - (PI * i as f32 / segments as f32);
        let x = angle.cos() * radius;
        let y = half_height + angle.sin() * radius;
        positions.push([x, y, 0.0]);
        normals.push([0.0, 0.0, 1.0]);
        uvs.push([0.5 + x / (radius * 2.0), 0.5 + y / ((half_height + radius) * 2.0)]);
    }

    // Bottom semicircle (from right to left)
    for i in 0..=segments {
        let angle = -(PI * i as f32 / segments as f32);
        let x = angle.cos() * radius;
        let y = -half_height + angle.sin() * radius;
        positions.push([x, y, 0.0]);
        normals.push([0.0, 0.0, 1.0]);
        uvs.push([0.5 + x / (radius * 2.0), 0.5 + y / ((half_height + radius) * 2.0)]);
    }

    // Create triangle fan indices
    let mut indices: Vec<u32> = Vec::new();
    let total_outer = (segments + 1) * 2;
    for i in 1..total_outer {
        indices.push(0); // Center
        indices.push(i as u32);
        indices.push((i % total_outer + 1) as u32);
    }
    // Close the fan
    indices.push(0);
    indices.push(total_outer as u32);
    indices.push(1);

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));

    mesh
}

/// Creates a 2D circle mesh.
///
/// # Arguments
/// * `radius` - Radius of the circle
/// * `segments` - Number of segments (more = smoother circle)
pub fn create_circle_mesh(radius: f32, segments: usize) -> Mesh {
    let segments = segments.max(8);

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(segments + 2);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(segments + 2);
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(segments + 2);

    // Center point
    positions.push([0.0, 0.0, 0.0]);
    normals.push([0.0, 0.0, 1.0]);
    uvs.push([0.5, 0.5]);

    // Circle vertices
    for i in 0..=segments {
        let angle = 2.0 * PI * i as f32 / segments as f32;
        let x = angle.cos() * radius;
        let y = angle.sin() * radius;
        positions.push([x, y, 0.0]);
        normals.push([0.0, 0.0, 1.0]);
        uvs.push([0.5 + x / (radius * 2.0), 0.5 + y / (radius * 2.0)]);
    }

    // Triangle fan indices
    let mut indices: Vec<u32> = Vec::new();
    for i in 1..=segments as u32 {
        indices.push(0);
        indices.push(i);
        indices.push(i + 1);
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));

    mesh
}

/// Creates a 2D triangle mesh from three vertices.
///
/// # Arguments
/// * `vertices` - Three 2D vertices defining the triangle
pub fn create_triangle_mesh(vertices: &[Vec2; 3]) -> Mesh {
    let positions: Vec<[f32; 3]> = vertices.iter().map(|v| [v.x, v.y, 0.0]).collect();
    let normals: Vec<[f32; 3]> = vec![[0.0, 0.0, 1.0]; 3];

    // Calculate bounding box for UV coordinates
    let min_x = vertices.iter().map(|v| v.x).fold(f32::INFINITY, f32::min);
    let max_x = vertices.iter().map(|v| v.x).fold(f32::NEG_INFINITY, f32::max);
    let min_y = vertices.iter().map(|v| v.y).fold(f32::INFINITY, f32::min);
    let max_y = vertices.iter().map(|v| v.y).fold(f32::NEG_INFINITY, f32::max);
    let width = max_x - min_x;
    let height = max_y - min_y;

    let uvs: Vec<[f32; 2]> = vertices
        .iter()
        .map(|v| {
            [
                if width > 0.0 { (v.x - min_x) / width } else { 0.5 },
                if height > 0.0 { (v.y - min_y) / height } else { 0.5 },
            ]
        })
        .collect();

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(vec![0, 1, 2]));

    mesh
}

/// Creates a 2D polygon mesh from vertices (fan triangulation from center).
///
/// # Arguments
/// * `vertices` - Vertices forming the polygon outline (in order)
pub fn create_polygon_mesh(vertices: &[Vec2]) -> Mesh {
    if vertices.len() < 3 {
        // Return a degenerate mesh for invalid input
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            bevy::render::render_asset::RenderAssetUsages::default(),
        );
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, Vec::<[f32; 3]>::new());
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, Vec::<[f32; 2]>::new());
        mesh.insert_indices(Indices::U32(vec![]));
        return mesh;
    }

    // Calculate center point for fan triangulation
    let center: Vec2 = vertices.iter().copied().sum::<Vec2>() / vertices.len() as f32;

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(vertices.len() + 1);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(vertices.len() + 1);
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(vertices.len() + 1);

    // Calculate bounding box for UV mapping
    let min_x = vertices.iter().map(|v| v.x).fold(f32::INFINITY, f32::min);
    let max_x = vertices.iter().map(|v| v.x).fold(f32::NEG_INFINITY, f32::max);
    let min_y = vertices.iter().map(|v| v.y).fold(f32::INFINITY, f32::min);
    let max_y = vertices.iter().map(|v| v.y).fold(f32::NEG_INFINITY, f32::max);
    let width = max_x - min_x;
    let height = max_y - min_y;

    // Center point
    positions.push([center.x, center.y, 0.0]);
    normals.push([0.0, 0.0, 1.0]);
    uvs.push([
        if width > 0.0 { (center.x - min_x) / width } else { 0.5 },
        if height > 0.0 { (center.y - min_y) / height } else { 0.5 },
    ]);

    // Outer vertices
    for v in vertices {
        positions.push([v.x, v.y, 0.0]);
        normals.push([0.0, 0.0, 1.0]);
        uvs.push([
            if width > 0.0 { (v.x - min_x) / width } else { 0.5 },
            if height > 0.0 { (v.y - min_y) / height } else { 0.5 },
        ]);
    }

    // Triangle fan indices
    let n = vertices.len() as u32;
    let mut indices: Vec<u32> = Vec::new();
    for i in 0..n {
        indices.push(0); // Center
        indices.push(i + 1);
        indices.push((i % n) + 2);
    }
    // Fix last triangle to wrap around
    if !indices.is_empty() {
        let last_idx = indices.len() - 1;
        indices[last_idx] = 1;
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));

    mesh
}

/// Creates a 2D rectangle mesh.
///
/// # Arguments
/// * `half_width` - Half the width of the rectangle
/// * `half_height` - Half the height of the rectangle
pub fn create_rectangle_mesh(half_width: f32, half_height: f32) -> Mesh {
    let positions: Vec<[f32; 3]> = vec![
        [-half_width, -half_height, 0.0], // Bottom-left
        [half_width, -half_height, 0.0],  // Bottom-right
        [half_width, half_height, 0.0],   // Top-right
        [-half_width, half_height, 0.0],  // Top-left
    ];

    let normals: Vec<[f32; 3]> = vec![[0.0, 0.0, 1.0]; 4];

    let uvs: Vec<[f32; 2]> = vec![
        [0.0, 0.0],
        [1.0, 0.0],
        [1.0, 1.0],
        [0.0, 1.0],
    ];

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3]));

    mesh
}
