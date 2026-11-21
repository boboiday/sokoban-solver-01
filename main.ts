// Sokoban solver — push-based A* with room decomposition, deadlock pruning and a
// minimum-assignment heuristic. Written from scratch for sokoban-solver-01.
// The generator builds each level as a hallway with 1-4 rooms off it; a box pushed
// out of its room can never come back, so every room is solved independently and
// the level's push-optimal count is the sum of the rooms' optima. Solving each
// 3-4 box room with push-A* (not move-A*) is what beats the plain-astar baseline,
// which optimises moves and therefore wastes pushes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS: [number, number, string][] = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
const OPPOSITE = [1, 0, 3, 2];

type Grid = {
  w: number;
  h: number;
  wall: boolean[];
  goals: number[];
  boxes: number[];
  player: number;
};

function parse(text: string): Grid {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall: boolean[] = new Array(w * h).fill(false);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = rows[y][x] ?? '#';
      const i = y * w + x;
      if (ch === '#') wall[i] = true;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS over floor cells from start, avoiding walls and the blocked set.
// Returns distance and parent arrays (parent for path reconstruction).
function bfs(
  start: number,
  wall: boolean[],
  blocked: Set<number>,
  w: number,
  h: number,
): { dist: number[]; parent: number[] } {
  const dist: number[] = new Array(w * h).fill(-1);
  const parent: number[] = new Array(w * h).fill(-1);
  const q: number[] = [start];
  dist[start] = 0;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const dx = DIRS[d]![0];
      const dy = DIRS[d]![1];
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || blocked.has(n) || dist[n] >= 0) continue;
      dist[n] = dist[c]! + 1;
      parent[n] = c;
      q.push(n);
    }
  }
  return { dist, parent };
}

// Walk path (list of direction letters) from start to target using a parent array.
function walkMoves(parent: number[], start: number, target: number, w: number): string {
  const cells: number[] = [];
  let c = target;
  while (c !== start && c !== -1) {
    cells.push(c);
    c = parent[c]!;
  }
  cells.reverse();
  let moves = '';
  let cur = start;
  for (const cell of cells) {
    const dx = (cell % w) - (cur % w);
    const dy = ((cell / w) | 0) - ((cur / w) | 0);
    for (let d = 0; d < 4; d++) {
      if (DIRS[d]![0] === dx && DIRS[d]![1] === dy) {
        moves += DIRS[d]![2];
        break;
      }
    }
    cur = cell;
  }
  return moves;
}

// Minimum-assignment heuristic: the cheapest way to match boxes to goals using
// precomputed push-distances. Admissible (a lower bound on the pushes left).
function assignmentHeuristic(
  boxes: number[],
  goalDist: number[][],
  nGoals: number,
): number {
  const n = boxes.length;
  if (n === 0) return 0;
  // For many boxes the permutation search explodes (n!); fall back to the
  // sum of each box's nearest-goal distance, which is still an admissible
  // lower bound (it just ignores assignment conflicts).
  if (n > 6) {
    let sum = 0;
    for (let i = 0; i < n; i++) {
      let m = Infinity;
      for (let g = 0; g < nGoals; g++) {
        const d = goalDist[g]![boxes[i]!]!;
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return 1e9;
      sum += m;
    }
    return sum;
  }
  // Brute force over goal permutations (n <= 6, so at most 720).
  const used: boolean[] = new Array(nGoals).fill(false);
  let best = Infinity;
  const dfs = (bi: number, cost: number): void => {
    if (cost >= best) return;
    if (bi === n) {
      best = cost;
      return;
    }
    for (let g = 0; g < nGoals; g++) {
      if (used[g]) continue;
      const d = goalDist[g]![boxes[bi]!]!;
      if (d < 0 || d === Infinity) continue;
      used[g] = true;
      dfs(bi + 1, cost + d);
      used[g] = false;
    }
  };
  dfs(0, 0);
  return best === Infinity ? 1e9 : best;
}

// Solve one room with push-based A*. roomCells: the room's floor cells (including
// the door cell). entry: the hallway cell just outside the door (walkable). The
// player starts on the door cell. Boxes are confined to roomCells. Returns the
// full move string (walks + pushes), or '' if unsolved.
function solveRoom(
  g: Grid,
  roomCells: Set<number>,
  roomBoxes: number[],
  roomGoals: number[],
  doorCell: number,
  entryCell: number,
): string {
  const { w, h, wall } = g;
  const goalSet = new Set(roomGoals);
  const nGoals = roomGoals.length;
  // Walkable area for the player: room cells + the entry cell. Confine the
  // search's walking to this area so the player never wanders into the
  // hallway or another room (whose boxes would be in the way).
  const walkable = (c: number): boolean =>
    !wall[c] && (roomCells.has(c) || c === entryCell);
  const roomWall: boolean[] = new Array(w * h).fill(true);
  for (let c = 0; c < w * h; c++) roomWall[c] = !walkable(c);
  // Dead cells: corners (two perpendicular walls) not on a goal. A box there
  // can never reach a goal. Precompute over room cells. The entry cell counts
  // as open floor, not a wall, so the door cell is never wrongly marked dead.
  const dead = new Set<number>();
  for (const c of roomCells) {
    if (goalSet.has(c)) continue;
    const x = c % w;
    const y = (c / w) | 0;
    const up = y === 0 || roomWall[c - w];
    const down = y === h - 1 || roomWall[c + w];
    const left = x === 0 || roomWall[c - 1];
    const right = x === w - 1 || roomWall[c + 1];
    if ((up || down) && (left || right)) dead.add(c);
  }
  // Goal push-distances: BFS from each goal over room cells (box moves).
  const goalDist: number[][] = [];
  for (const goal of roomGoals) {
    const dist: number[] = new Array(w * h).fill(-1);
    const q: number[] = [goal];
    dist[goal] = 0;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi]!;
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DIRS[d]![0];
        const ny = y + DIRS[d]![1];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const n = ny * w + nx;
        if (!roomCells.has(n) || dist[n] >= 0) continue;
        dist[n] = dist[c]! + 1;
        q.push(n);
      }
    }
    goalDist.push(dist);
  }
  // 2x2 deadlock: a 2x2 block of walls/boxes (no goal in it) can never be
  // undone. Check every 2x2 block whose top-left corner is a room cell.
  const is2x2Dead = (boxes: number[]): boolean => {
    const bset = new Set(boxes);
    const solid = (c: number): boolean =>
      wall[c] || !roomCells.has(c) || bset.has(c);
    for (const c of roomCells) {
      const x = c % w;
      const y = (c / w) | 0;
      if (x >= w - 1 || y >= h - 1) continue;
      const a = c;
      const b = c + 1;
      const d = c + w;
      const e = c + w + 1;
      if (goalSet.has(a) || goalSet.has(b) || goalSet.has(d) || goalSet.has(e)) continue;
      if (solid(a) && solid(b) && solid(d) && solid(e)) return true;
    }
    return false;
  };
  const key = (p: number, bs: number[]): string => p + ':' + bs.join(',');
  const startBoxes = roomBoxes.slice().sort((a, b) => a - b);
  const startPlayer = doorCell;
  // A* state arrays.
  const sp: number[] = [startPlayer];
  const sb: number[][] = [startBoxes];
  const sg: number[] = [0];
  const sparent: number[] = [-1];
  const smove: string[] = [''];
  const best = new Map<string, number>([[key(startPlayer, startBoxes), 0]]);
  // Heap of [f, g, stateIndex].
  const heap: [number, number, number][] = [];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && a[1] > b[1]);
  const hpush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      const t = heap[i]!;
      heap[i] = heap[p]!;
      heap[p] = t;
      i = p;
    }
  };
  const hpop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        const t = heap[i]!;
        heap[i] = heap[c]!;
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };
  const h0 = assignmentHeuristic(startBoxes, goalDist, nGoals);
  hpush([h0, 0, 0]);
  const MAX_EXPAND = 400000;
  let expanded = 0;
  const canReachDoor = (player: number, boxes: number[]): boolean => {
    const blocked = new Set(boxes);
    const { dist } = bfs(player, roomWall, blocked, w, h);
    return dist[doorCell] >= 0;
  };
  while (heap.length > 0 && expanded < MAX_EXPAND) {
    const [, gg, s] = hpop();
    if (best.get(key(sp[s]!, sb[s]!)) !== s) continue;
    const bs = sb[s]!;
    const p = sp[s]!;
    // Goal: every box on a goal, and the player can still reach the door.
    if (bs.every((b) => goalSet.has(b))) {
      if (canReachDoor(p, bs)) {
        let moves = '';
        for (let t = s; t > 0; t = sparent[t]!) moves = smove[t]! + moves;
        return moves;
      }
      continue;
    }
    expanded++;
    // Player reachable region (without moving boxes), confined to the room.
    const blocked = new Set(bs);
    const { parent } = bfs(p, roomWall, blocked, w, h);
    const reachable = new Set<number>();
    for (let c = 0; c < w * h; c++) if (parent[c] >= 0 || c === p) reachable.add(c);
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const dx = DIRS[d]![0];
        const dy = DIRS[d]![1];
        const behind = (by - dy) * w + (bx - dx);
        const beyond = (by + dy) * w + (bx + dx);
        if (!walkable(behind) || !reachable.has(behind)) continue;
        if (!roomCells.has(beyond)) continue;
        if (bs.includes(beyond)) continue;
        if (dead.has(beyond)) continue;
        const nb = bs.slice();
        nb[bi] = beyond;
        nb.sort((a, b) => a - b);
        if (is2x2Dead(nb)) continue;
        const np = b;
        const k = key(np, nb);
        const seen = best.get(k);
        if (seen !== undefined && sg[seen]! <= gg + 1) continue;
        // Walk to behind, then push.
        const walk = walkMoves(parent, p, behind, w);
        const move = walk + DIRS[d]![2].toUpperCase();
        const t = sp.length;
        sp.push(np);
        sb.push(nb);
        sg.push(gg + 1);
        sparent.push(s);
        smove.push(move);
        best.set(k, t);
        const hh = assignmentHeuristic(nb, goalDist, nGoals);
        hpush([gg + 1 + hh, gg + 1, t]);
      }
    }
  }
  return '';
}

// Solve a whole level by decomposing it into rooms off the hallway.
function solve(text: string): string {
  const g = parse(text);
  const { w, h, wall } = g;
  const goalSet = new Set(g.goals);
  const playerRow = (g.player / w) | 0;
  // Rooms: connected components of floor cells excluding the hallway row.
  const seen = new Set<number>();
  type Room = {
    cells: Set<number>;
    boxes: number[];
    goals: number[];
    door: number;
    entry: number;
  };
  const rooms: Room[] = [];
  for (let c = 0; c < w * h; c++) {
    if (wall[c] || ((c / w) | 0) === playerRow || seen.has(c)) continue;
    // Flood fill this component.
    const cells = new Set<number>();
    const q: number[] = [c];
    seen.add(c);
    cells.add(c);
    for (let qi = 0; qi < q.length; qi++) {
      const cur = q[qi]!;
      const x = cur % w;
      const y = (cur / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DIRS[d]![0];
        const ny = y + DIRS[d]![1];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        if (ny === playerRow) continue;
        const n = ny * w + nx;
        if (wall[n] || seen.has(n)) continue;
        seen.add(n);
        cells.add(n);
        q.push(n);
      }
    }
    const boxes = g.boxes.filter((b) => cells.has(b));
    const goals = g.goals.filter((gl) => cells.has(gl));
    if (boxes.length === 0 && goals.length === 0) continue;
    // Door: the cell in the room adjacent to the hallway row.
    let door = -1;
    let entry = -1;
    for (const cell of cells) {
      const x = cell % w;
      const y = (cell / w) | 0;
      const ny = y < playerRow ? y + 1 : y - 1;
      if (ny === playerRow) {
        const n = ny * w + x;
        if (!wall[n]) {
          door = cell;
          entry = n;
          break;
        }
      }
    }
    if (door < 0) continue;
    rooms.push({ cells, boxes, goals, door, entry });
  }
  // If decomposition failed (no rooms found), fall back to treating the whole
  // level as one room entered from the player's start.
  if (rooms.length === 0) {
    const cells = new Set<number>();
    for (let c = 0; c < w * h; c++) if (!wall[c]) cells.add(c);
    const moves = solveRoom(g, cells, g.boxes, g.goals, g.player, g.player);
    return moves;
  }
  // Visit rooms in reading order (top to bottom, left to right).
  rooms.sort((a, b) => a.door - b.door);
  let moves = '';
  let player = g.player;
  let curBoxes = g.boxes.slice();
  for (const room of rooms) {
    // Walk to the hallway cell outside this room's door.
    const blocked = new Set(curBoxes);
    const { parent } = bfs(player, wall, blocked, w, h);
    if (parent[room.entry] >= 0 || player === room.entry) {
      moves += walkMoves(parent, player, room.entry, w);
      player = room.entry;
    } else {
      // Cannot reach this room's door: give up on the level.
      return '';
    }
    // Step into the door.
    const dx = (room.door % w) - (player % w);
    const dy = ((room.door / w) | 0) - ((player / w) | 0);
    let stepLetter = '';
    for (let d = 0; d < 4; d++) {
      if (DIRS[d]![0] === dx && DIRS[d]![1] === dy) {
        stepLetter = DIRS[d]![2];
        break;
      }
    }
    if (!stepLetter) return '';
    moves += stepLetter;
    player = room.door;
    // Solve the room. The room's boxes are the ones currently in it.
    const roomMoves = solveRoom(g, room.cells, room.boxes, room.goals, room.door, room.entry);
    if (!roomMoves) return '';
    moves += roomMoves;
    // Update the global box positions: the room's boxes are now on its goals.
    // Recompute the player position by replaying is unnecessary: the room
    // solver's returned moves end with the player able to reach the door, so
    // walk out through the door to the hallway for the next room.
    // Track the player's end position by simulating the room moves on a copy.
    const sim = simulate(g, player, room.boxes, roomMoves);
    if (!sim) return '';
    player = sim.player;
    // Boxes are indistinguishable: replace the room's initial boxes (as a set)
    // with the simulated final positions.
    const roomBoxSet = new Set(room.boxes);
    curBoxes = curBoxes.filter((b) => !roomBoxSet.has(b)).concat(sim.boxes);
    curBoxes.sort((a, b) => a - b);
    // Walk out of the room to the hallway (through the door) for the next room.
    const blocked2 = new Set(curBoxes);
    const out = bfs(player, wall, blocked2, w, h);
    if (out.parent[room.entry] >= 0 || player === room.entry) {
      moves += walkMoves(out.parent, player, room.entry, w);
      player = room.entry;
    }
  }
  // Sanity: every box must be on a goal.
  if (!curBoxes.every((b) => goalSet.has(b))) return '';
  return moves;
}

// Simulate a move string from a given player/box state (for tracking the
// player's end position). Returns null on an illegal move.
function simulate(
  g: Grid,
  player: number,
  boxes: number[],
  moves: string,
): { player: number; boxes: number[] } | null {
  const { w, wall } = g;
  let p = player;
  const bs = boxes.slice();
  for (const ch of moves) {
    const letter = ch.toLowerCase();
    let d = -1;
    for (let i = 0; i < 4; i++) if (DIRS[i]![2] === letter) d = i;
    if (d < 0) continue;
    const dx = DIRS[d]![0];
    const dy = DIRS[d]![1];
    const q = p + dy * w + dx;
    if (wall[q]) return null;
    const at = bs.indexOf(q);
    if (at >= 0) {
      const beyond = q + dy * w + dx;
      if (wall[beyond] || bs.includes(beyond)) return null;
      bs[at] = beyond;
    }
    p = q;
  }
  bs.sort((a, b) => a - b);
  return { player: p, boxes: bs };
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
