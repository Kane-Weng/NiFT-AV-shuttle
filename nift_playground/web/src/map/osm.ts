/*
 * File: osm.ts
 *
 * Description:
 *   A small Lanelet2 .osm reader for the page. It reads nodes, ways and relations with their
 *   tags, without a DOM, so it runs in the page, in a worker and in Node tests.
 *   It parses the 4 MB odr2024 map in well under a second.
 */

export type Tags = Record<string, string>

export interface OsmNode {
  id: number
  lat: number
  lon: number
}

export interface OsmWay {
  id: number
  nodes: number[]
  tags: Tags
}

export interface OsmMember {
  type: string
  ref: number
  role: string
}

export interface OsmRelation {
  id: number
  members: OsmMember[]
  tags: Tags
}

export interface OsmData {
  nodes: Map<number, OsmNode>
  ways: Map<number, OsmWay>
  relations: Map<number, OsmRelation>
}

const ATTR_RE = /([A-Za-z_:]+)\s*=\s*(?:'([^']*)'|"([^"]*)")/g
const ELEMENT_RE = /<(node|way|relation)\b([^>]*?)(\/?)>|<\/(way|relation|node)>|<(nd|tag|member)\b([^>]*?)\/?>/g

function attrs(text: string): Record<string, string> {
  const out: Record<string, string> = {}
  ATTR_RE.lastIndex = 0
  let m: RegExpExecArray | null
  while ((m = ATTR_RE.exec(text)) !== null) out[m[1]] = m[2] ?? m[3] ?? ''
  return out
}

/** Parses Lanelet2 .osm text. Elements with action='delete' are skipped, like JOSM files. */
export function parseOsm(text: string): OsmData {
  const data: OsmData = { nodes: new Map(), ways: new Map(), relations: new Map() }
  let way: OsmWay | null = null
  let rel: OsmRelation | null = null
  let tagsTarget: Tags | null = null
  let skip = false

  ELEMENT_RE.lastIndex = 0
  let m: RegExpExecArray | null
  while ((m = ELEMENT_RE.exec(text)) !== null) {
    if (m[1]) {
      const a = attrs(m[2])
      const selfClosing = m[3] === '/'
      skip = a.action === 'delete'
      const id = Number(a.id)
      if (m[1] === 'node') {
        if (!skip) data.nodes.set(id, { id, lat: Number(a.lat), lon: Number(a.lon) })
        tagsTarget = null
      } else if (m[1] === 'way') {
        way = { id, nodes: [], tags: {} }
        tagsTarget = way.tags
        if (selfClosing) {
          if (!skip) data.ways.set(id, way)
          way = null
        }
      } else {
        rel = { id, members: [], tags: {} }
        tagsTarget = rel.tags
        if (selfClosing) {
          if (!skip) data.relations.set(id, rel)
          rel = null
        }
      }
    } else if (m[4]) {
      if (m[4] === 'way' && way) {
        if (!skip) data.ways.set(way.id, way)
        way = null
      } else if (m[4] === 'relation' && rel) {
        if (!skip) data.relations.set(rel.id, rel)
        rel = null
      }
      tagsTarget = null
    } else if (m[5]) {
      const a = attrs(m[6])
      if (m[5] === 'nd' && way) way.nodes.push(Number(a.ref))
      else if (m[5] === 'member' && rel) rel.members.push({ type: a.type, ref: Number(a.ref), role: a.role ?? '' })
      else if (m[5] === 'tag' && tagsTarget) tagsTarget[a.k] = a.v
    }
  }
  return data
}
