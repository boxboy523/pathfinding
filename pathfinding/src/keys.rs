//! Typed generational handles into a Map's topology stores.
//! Keys belong to their originating Map. Local terrain edits invalidate only removed
//! objects; unaffected objects retain their keys. Flow validity still uses Map generation.
slotmap::new_key_type! {
    pub struct BlockId;
    pub struct PortalId;
}
