import bpy

scene = bpy.context.scene
node_tree = scene.node_tree

node_1 = node_tree.nodes.new("CompositorNodeRLayers")
node_1.location = (-200, 20)

node_2 = node_tree.nodes.new("CompositorNodeNormalize")
node_2.location = (200, -20)

node_3 = node_tree.nodes.new("CompositorNodeOutputFile")
node_3.location = (400, 20)
node_3.base_path = "/home/hoang/OSS/VoteGrasp/blender-scripts/depth/"
bpy.data.scenes["Scene"].node_tree.nodes['File Output'].format.color_depth = '16'
bpy.data.scenes["Scene"].node_tree.nodes['File Output'].format.color_mode = 'BW'


node_tree.links.new(node_1.outputs["Depth"], node_2.inputs[0])
node_tree.links.new(node_2.outputs[0], node_3.inputs["Image"])

# render
for i in range(0,2):
    bpy.ops.render.render()

