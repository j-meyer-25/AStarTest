//////////////////////////////////////////////////
// Fixed-Size Rust A* Implementation (Unstable) //
// Josh Meyer, Alex Burkholder                  //
// Spring 2024                                  //
//////////////////////////////////////////////////

const COLS: usize = 8;
const ROWS: usize = 8;
const SUCCESSOR_DIRS: [[i8; 2]; 4] = [
    [1,0], [0,1], [0,-1], [-1, 0],
];

// Data structure to represent cells
#[derive(Copy, Clone)]
struct Node {
    row: i32,
    col: i32,
    row_parent: i32,
    col_parent: i32,
    f: f64,
    g: f64,
    h: f64,
}

/* 
Verifies if cell is occupied
*/
fn is_blocked(graph: &[[i32; COLS]; ROWS], row: i32, col: i32) -> bool {
    return graph[row as usize][col as usize] == 0;
}

/* 
Verifies if cell is not out-of-bounds
*/
fn is_valid(row: i32, col: i32) -> bool {
    return row >= 0 && row < ROWS as i32 && col >= 0 && col < COLS as i32;
}

/* 
Verifies if cell is the 'goal' or end node
*/
fn is_end(row: i32, col: i32, end_node: &Node) -> bool {
    return end_node.row == row && end_node.col == col;
}

/* 
Herustic function - Euclidean implementation
May require changing
*/
fn get_heuristic(row: i32, col: i32, dest: &Node) -> f64 {
    return f64::powf(f64::powf( (row - dest.row) as f64, 2.0) + f64::powf( (col - dest.col) as f64, 2.0), 0.5)
}

/* 
Verifies if a given row+col combination is within a vector of nodes
*/
fn pos_is_in_list(row: i32, col: i32, list: &Vec<Node>) -> bool {
    for maybe_node in list {
        if row == maybe_node.row && col == maybe_node.col { return true; }
    }
    return false;
}

/* 
Given a completed node-graph, prints full path from start to end
Generally assumes that a path exists
*/
fn print_path(node_graph: &Vec<Vec<Node>>, start: &Node, end: &Node) {
    let mut path_trace: Vec<Node> = vec![];
    let mut cur_node = end;
    while cur_node.row != start.row && cur_node.col != start.col {
        path_trace.push(*cur_node);
        cur_node = &node_graph[cur_node.row_parent as usize][cur_node.col_parent as usize];
    }
    path_trace.reverse();
    for node in path_trace {
        eprint!("({}, {}) -> ", node.row, node.col);
    }
}

/* 
Primary A* Algorithm Implementation
Graph is a 2D fix-sized array of binary values
Start and End are nodes to route through
*/
fn astar_find(graph: &[[i32; COLS]; ROWS], start: Node, end: Node) {
    if is_blocked(graph, start.row, start.col) || is_blocked(graph, end.row, end.col) { return; }
    if !is_valid(start.row, start.col) || !is_valid(end.row, end.col) { return; }

    let mut open_list: Vec<Node> = vec![];
    let mut closed_list: Vec<Node> = vec![];
    open_list.push(start);
    
    // Reconstruction of 'graph' using Nodes
    let mut node_graph: Vec<Vec<Node>> = vec![];
    for (i, row_list) in graph.iter().enumerate() {
        node_graph.push( vec![] );
        for (j, _) in row_list.iter().enumerate() {
            node_graph[i].push( Node{row: i as i32, col: j as i32, row_parent: i as i32, col_parent: j as i32, f: f64::MAX, g: f64::MAX, h: 0.0} )
        }
    }
    
    while open_list.len() > 0 {
        // Find the node with the lowest 'f' in open list, pop it
        let mut least_f: f64 = f64::MAX;
        let mut least_ind = 0 as usize;
        for (i, node) in open_list.iter().enumerate() {
            if node.f < least_f {
                least_f = node.f;
                least_ind = i;
            }
        }
        let lowest = open_list.swap_remove(least_ind);
        
        closed_list.push(lowest);
        
        // Look at all adjacent directions
        for dir in SUCCESSOR_DIRS {
            let new_row = lowest.row + dir[0] as i32;
            let new_col = lowest.col + dir[1] as i32;
    
            if !is_valid(new_row, new_col) { continue; }
            if is_blocked(graph, new_row, new_col) { continue; }
            if pos_is_in_list(new_row, new_col, &closed_list) { continue; }
            
            let found_node = &node_graph[new_row as usize][new_col as usize];
  
            if is_end(new_row, new_col, &end) {
                node_graph[new_row as usize][new_col as usize].row_parent = lowest.row;
                node_graph[new_row as usize][new_col as usize].col_parent = lowest.col;
                eprint!("Successfully got to destination.\n");
                print_path(&node_graph, &start, &end);
                return;
            } else {
                let g_new = node_graph[lowest.row as usize][lowest.col as usize].g + 1.0;
                let h_new = get_heuristic(new_row, new_col, &found_node);
                let f_new = g_new + h_new;

                if node_graph[new_row as usize][new_col as usize].f == f64::MAX || node_graph[new_row as usize][new_col as usize].f > f_new {
                    open_list.push( Node{row: new_row, col: new_col, row_parent: lowest.row, col_parent: lowest.col, f: f_new, g: g_new, h: h_new} );
                    
                    node_graph[new_row as usize][new_col as usize].f = f_new;
                    node_graph[new_row as usize][new_col as usize].g = g_new;
                    node_graph[new_row as usize][new_col as usize].h = h_new;
                    node_graph[new_row as usize][new_col as usize].row_parent = lowest.row;
                    node_graph[new_row as usize][new_col as usize].col_parent = lowest.col;
                }
            }
            //let this_cost;
        }
    }
    eprint!("Could not find the destination.\n");
}

/* 
Main
*/
fn main() {
    let graph: [[i32; COLS]; ROWS] = [
        [1, 1, 1, 1, 1, 0, 1, 1],
        [1, 0, 1, 0, 1, 1, 1, 0],
        [1, 1, 1, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0, 1, 0],
        [0, 1, 1, 1, 0, 0, 1, 0],
        [1, 0, 0, 1, 0, 1, 1, 0],
        [1, 0, 1, 1, 0, 1, 0, 1],
        [0, 1, 1, 1, 1, 1, 0, 1]
    ];
    
    for row in graph {
        println!("{:?}", row);
    }
    
    eprint!("Attempting to search...\n");
    astar_find(&graph,
        Node{row: 0, col: 1, row_parent: 0, col_parent: 1, f: 0.0, g: 0.0, h: 0.0},
        Node{row: 3, col: 6, row_parent: 3, col_parent: 6, f: 0.0, g: 0.0, h: 0.0}
    );
    eprint!("Done.\n"); 
}