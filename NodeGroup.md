// "Semi-Dynamic"
```
class StereoCombined : NodeGroup {
    StereoCombined() {
        auto stereoHorizontal = create<node::StereoDepth>();
        auto stereoVertical = create<node::StereoDepth>();
        auto sync = create<node::Sync>();

        stereoHorizontal->outDepth.link(sync->inputs['horizontal']);
        stereoVertical->outDepth.link(sync->inputs['vertical']);
    }
}
```

// Static
```
class StereoCombined : NodeGroup {
    StereoDepth stereoHorizontal;
    StereoDepth stereoVertical;
    Sync sync;

    StereoCombined() : out(sync.out) {
        add(stereoHorizontal);
        add(stereoVertical);
        add(sync);

        stereoHorizontal.outDepth.link(sync.inputs['horizontal']);
        stereoVertical.outDepth.link(sync.inputs['vertical']);
    }

    Output& out;
}
```

// Dynamic dynamic
```
NodeGroup grp;
// Create
auto stereoHorizontal = grp.create<node::StereoDepth>();
auto stereoVertical = grp.create<node::StereoDepth>();
auto sync = grp.create<node::Sync>();
// Link
stereoHorizontal->outDepth.link(sync->inputs['horizontal']);
stereoVertical->outDepth.link(sync->inputs['vertical']);
```
// Modified dynamic dynamic - prevent stack usage of NodeGroup (unless we impl. stack compatible shared ptr)
```
auto grp = std::make_shared<NodeGroup>();
// Create
auto stereoHorizontal = grp->create<node::StereoDepth>();
auto stereoVertical = grp->create<node::StereoDepth>();
auto sync = grp->create<node::Sync>();
// Link
stereoHorizontal->outDepth.link(sync->inputs['horizontal']);
stereoVertical->outDepth.link(sync->inputs['vertical']);
```


// TODO(themarpe)
// Add capability of adding IOs dynamically as well (to the edges of the Group)



### Notes

Linking happens at first possible level. (eg Node A-> Node B, Node C - if linking in Node C, then linking happens inside Node C, if linking between C & B, linking happens in Node A)




// Combines elements from both Node & Pipeline.
// Acts like a Node (has IO), but also has capability to place other nodes or node groups and link


// create(Node) function
// create(NodeGroup)
// link
// unlink
// remove
// ...
// group name (optional)
//
// IO and getInputRef / getInputMapsRefs...



// Options:
// 1. Recursive Node /w pipeline capabilities
// 2. Recursive Pipeline /w node capabilities
// 3. NodeGroup, new concept /w Node & Pipeline capabilities


// assets:
// /pipeline
// /node/{id}/{id...}/
// /{id}/

