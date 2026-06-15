from dataclasses import dataclass
from typing import Dict, List, Set, Tuple

import cv2
import depthai as dai
import numpy as np


@dataclass
class MessageGroupRenderNode:
    nodeIndex: int = 0
    depth: int = 0
    row: int = 0
    isRoot: bool = False
    isUnreachable: bool = False
    label: str = ""
    annotation: str = ""


@dataclass
class MessageGroupRenderEdge:
    parentNodeIndex: int = 0
    childNodeIndex: int = 0
    itemIndex: int = 0
    isShared: bool = False
    isCycle: bool = False


@dataclass
class MessageGroupRenderGraph:
    nodes: List[MessageGroupRenderNode]
    edges: List[MessageGroupRenderEdge]
    treeText: str


def formatMessageNodeLabel(messageGroup: dai.MessageGroup, nodeIndex: int) -> str:
    node = messageGroup.getNode(nodeIndex)
    label = f"[{nodeIndex}] "

    if node is None:
        return label + "<missing>"

    return label + type(node).__name__


def buildMessageGroupRenderGraph(messageGroup: dai.MessageGroup) -> MessageGroupRenderGraph:
    graph = MessageGroupRenderGraph(nodes=[], edges=[], treeText="")
    allMessageIndices = messageGroup.getMessageIndices()
    if not allMessageIndices:
        graph.treeText = "<empty>\n"
        return graph

    sortedRoots = sorted(messageGroup.getRootMessageNodes())
    if not sortedRoots:
        sortedRoots = allMessageIndices

    output: List[str] = []
    renderedNodes: Set[int] = set()
    activePath: Set[int] = set()
    nodePositions: Dict[int, int] = {}

    def addGraphNode(
        nodeIndex: int,
        depth: int,
        isRoot: bool,
        isUnreachable: bool,
        annotation: str = "",
    ) -> None:
        existing = nodePositions.get(nodeIndex)
        if existing is not None:
            node = graph.nodes[existing]
            node.depth = min(node.depth, depth)
            node.isRoot = node.isRoot or isRoot
            node.isUnreachable = node.isUnreachable or isUnreachable
            if not node.annotation:
                node.annotation = annotation
            return

        node = MessageGroupRenderNode(
            nodeIndex=nodeIndex,
            depth=depth,
            row=len(graph.nodes),
            isRoot=isRoot,
            isUnreachable=isUnreachable,
            label=formatMessageNodeLabel(messageGroup, nodeIndex),
            annotation=annotation,
        )
        nodePositions[nodeIndex] = len(graph.nodes)
        graph.nodes.append(node)

    def appendChildren(nodeIndex: int, prefix: str, depth: int) -> None:
        childLinks = messageGroup.getLinksFromParent(nodeIndex)
        childLinks.sort(key=lambda link: (link.itemIndex, link.childNodeIndex))

        for childIndex, link in enumerate(childLinks):
            isLastChild = childIndex + 1 == len(childLinks)
            isCycle = link.childNodeIndex in activePath
            isFirstRender = not isCycle and link.childNodeIndex not in renderedNodes
            isShared = not isCycle and not isFirstRender

            graph.edges.append(
                MessageGroupRenderEdge(
                    parentNodeIndex=nodeIndex,
                    childNodeIndex=link.childNodeIndex,
                    itemIndex=link.itemIndex,
                    isShared=isShared,
                    isCycle=isCycle,
                )
            )

            line = (
                prefix
                + ("`-- " if isLastChild else "|-- ")
                + f"ItemIndex={link.itemIndex} -> "
                + formatMessageNodeLabel(messageGroup, link.childNodeIndex)
            )

            if isCycle:
                output.append(line + " (cycle)\n")
                continue

            if not isFirstRender:
                output.append(line + " (shared)\n")
                continue

            output.append(line + "\n")
            renderedNodes.add(link.childNodeIndex)
            addGraphNode(link.childNodeIndex, depth + 1, False, False)
            activePath.add(link.childNodeIndex)
            appendChildren(link.childNodeIndex, prefix + ("    " if isLastChild else "|   "), depth + 1)
            activePath.remove(link.childNodeIndex)

    wroteAnyRoot = False
    for rootIndex in sortedRoots:
        if rootIndex in renderedNodes:
            continue

        if wroteAnyRoot:
            output.append("\n")

        wroteAnyRoot = True
        renderedNodes.add(rootIndex)
        activePath.add(rootIndex)
        addGraphNode(rootIndex, 0, True, False)
        output.append(formatMessageNodeLabel(messageGroup, rootIndex) + "\n")
        appendChildren(rootIndex, "", 0)
        activePath.remove(rootIndex)

    sortedAllIndices = sorted(allMessageIndices)
    for nodeIndex in sortedAllIndices:
        if nodeIndex in renderedNodes:
            continue

        if wroteAnyRoot:
            output.append("\n")

        wroteAnyRoot = True
        renderedNodes.add(nodeIndex)
        activePath.add(nodeIndex)
        addGraphNode(nodeIndex, 0, True, True, "unreachable-from-roots")
        output.append(formatMessageNodeLabel(messageGroup, nodeIndex) + " (unreachable-from-roots)\n")
        appendChildren(nodeIndex, "", 0)
        activePath.remove(nodeIndex)

    graph.treeText = "".join(output)
    return graph


def visualizeMessageGroupTree(messageGroup: dai.MessageGroup) -> str:
    return buildMessageGroupRenderGraph(messageGroup).treeText


def renderMessageGroupTreeFrame(messageGroup: dai.MessageGroup, title: str = "MessageGroup tree") -> np.ndarray:
    graph = buildMessageGroupRenderGraph(messageGroup)

    kMarginX = 24
    kMarginY = 24
    kTopPadding = 18
    kBottomPadding = 24
    kNodePaddingX = 14
    kNodePaddingY = 10
    kColumnGap = 80
    kRowGap = 22
    kMinFrameWidth = 720
    kMinFrameHeight = 180
    kTitleGap = 24

    kFontFace = cv2.FONT_HERSHEY_SIMPLEX
    kTitleScale = 0.7
    kLabelScale = 0.5
    kAnnotationScale = 0.42
    kEdgeScale = 0.42
    kTitleThickness = 2
    kTextThickness = 1

    titleSize, _ = cv2.getTextSize(title, kFontFace, kTitleScale, kTitleThickness)

    if not graph.nodes:
        width = max(kMinFrameWidth, titleSize[0] + (2 * kMarginX))
        height = max(kMinFrameHeight, titleSize[1] + 140)
        frame = np.full((height, width, 3), (247, 247, 244), dtype=np.uint8)

        cv2.putText(frame, title, (kMarginX, kMarginY + titleSize[1]), kFontFace, kTitleScale, (35, 35, 35), kTitleThickness)
        cv2.putText(frame, "<empty>", (kMarginX, kMarginY + titleSize[1] + kTitleGap + 32), kFontFace, 0.6, (90, 90, 90), kTextThickness)
        return frame

    maxNodeLabelWidth = 0
    maxNodeLabelHeight = 0
    maxNodeAnnotationWidth = 0
    maxNodeAnnotationHeight = 0
    maxEdgeLabelWidth = 0
    maxDepth = 0
    maxRow = 0

    for node in graph.nodes:
        labelSize, _ = cv2.getTextSize(node.label, kFontFace, kLabelScale, kTextThickness)
        maxNodeLabelWidth = max(maxNodeLabelWidth, labelSize[0])
        maxNodeLabelHeight = max(maxNodeLabelHeight, labelSize[1])

        if node.annotation:
            annotationSize, _ = cv2.getTextSize(node.annotation, kFontFace, kAnnotationScale, kTextThickness)
            maxNodeAnnotationWidth = max(maxNodeAnnotationWidth, annotationSize[0])
            maxNodeAnnotationHeight = max(maxNodeAnnotationHeight, annotationSize[1])

        maxDepth = max(maxDepth, node.depth)
        maxRow = max(maxRow, node.row)

    for edge in graph.edges:
        edgeLabel = str(edge.itemIndex)
        if edge.isCycle:
            edgeLabel += " cycle"
        elif edge.isShared:
            edgeLabel += " shared"
        edgeLabelSize, _ = cv2.getTextSize(edgeLabel, kFontFace, kEdgeScale, kTextThickness)
        maxEdgeLabelWidth = max(maxEdgeLabelWidth, edgeLabelSize[0])

    nodeWidth = max(220, max(maxNodeLabelWidth, maxNodeAnnotationWidth) + (2 * kNodePaddingX))
    nodeHeight = max(
        50,
        maxNodeLabelHeight + maxNodeAnnotationHeight + (8 if maxNodeAnnotationHeight > 0 else 0) + (2 * kNodePaddingY),
    )
    contentTop = kMarginY + titleSize[1] + kTitleGap + kTopPadding
    width = max(
        kMinFrameWidth,
        (2 * kMarginX) + ((maxDepth + 1) * nodeWidth) + (maxDepth * kColumnGap) + maxEdgeLabelWidth + 32,
    )
    height = max(
        kMinFrameHeight,
        contentTop + ((maxRow + 1) * nodeHeight) + (maxRow * kRowGap) + kBottomPadding,
    )

    frame = np.full((height, width, 3), (247, 247, 244), dtype=np.uint8)

    cv2.putText(frame, title, (kMarginX, kMarginY + titleSize[1]), kFontFace, kTitleScale, (35, 35, 35), kTitleThickness)

    nodeRects: Dict[int, Tuple[int, int, int, int]] = {}
    for node in graph.nodes:
        x = kMarginX + (node.depth * (nodeWidth + kColumnGap))
        y = contentTop + (node.row * (nodeHeight + kRowGap))
        nodeRects[node.nodeIndex] = (x, y, nodeWidth, nodeHeight)

    for edge in graph.edges:
        parentRect = nodeRects.get(edge.parentNodeIndex)
        childRect = nodeRects.get(edge.childNodeIndex)
        if parentRect is None or childRect is None:
            continue

        parentX, parentY, parentWidth, parentHeight = parentRect
        childX, childY, childWidth, childHeight = childRect

        start = (parentX + parentWidth, parentY + (parentHeight // 2))
        end = (childX, childY + (childHeight // 2))
        elbowX = start[0] + max(24, (end[0] - start[0]) // 2)

        edgeColor = (115, 115, 115)
        if edge.isCycle:
            edgeColor = (45, 65, 210)
        elif edge.isShared:
            edgeColor = (0, 140, 220)

        cv2.line(frame, start, (elbowX, start[1]), edgeColor, 2, cv2.LINE_AA)
        cv2.line(frame, (elbowX, start[1]), (elbowX, end[1]), edgeColor, 2, cv2.LINE_AA)
        cv2.line(frame, (elbowX, end[1]), end, edgeColor, 2, cv2.LINE_AA)

        edgeLabel = str(edge.itemIndex)
        if edge.isCycle:
            edgeLabel += " cycle"
        elif edge.isShared:
            edgeLabel += " shared"

        labelX = min(elbowX + 6, frame.shape[1] - 4)
        labelY = max(min(((start[1] + end[1]) // 2) - 4, frame.shape[0] - 10), 18)
        cv2.putText(frame, edgeLabel, (labelX, labelY), kFontFace, kEdgeScale, edgeColor, kTextThickness)

    for node in graph.nodes:
        rect = nodeRects[node.nodeIndex]
        x, y, rectWidth, rectHeight = rect

        fillColor = (229, 236, 245)
        borderColor = (79, 100, 136)
        if node.isUnreachable:
            fillColor = (220, 232, 245)
            borderColor = (52, 102, 168)
        elif node.isRoot:
            fillColor = (214, 238, 219)
            borderColor = (58, 128, 71)

        cv2.rectangle(frame, (x, y), (x + rectWidth, y + rectHeight), fillColor, cv2.FILLED, cv2.LINE_AA)
        cv2.rectangle(frame, (x, y), (x + rectWidth, y + rectHeight), borderColor, 2, cv2.LINE_AA)

        currentY = y + kNodePaddingY + maxNodeLabelHeight
        cv2.putText(frame, node.label, (x + kNodePaddingX, currentY), kFontFace, kLabelScale, (30, 30, 30), kTextThickness)

        if node.annotation:
            currentY += maxNodeAnnotationHeight + 8
            cv2.putText(
                frame,
                node.annotation,
                (x + kNodePaddingX, currentY),
                kFontFace,
                kAnnotationScale,
                (75, 75, 75),
                kTextThickness,
            )

    return frame


def showMessageGroupTreeIfChanged(
    messageGroup: dai.MessageGroup,
    previousVisualization: str,
    windowName: str = "MessageGroup tree",
) -> Tuple[bool, str]:
    graph = buildMessageGroupRenderGraph(messageGroup)
    if graph.treeText == previousVisualization:
        return False, previousVisualization

    previousVisualization = graph.treeText
    cv2.imshow(windowName, renderMessageGroupTreeFrame(messageGroup, windowName))
    return True, previousVisualization


def printMessageGroupTreeIfChanged(
    messageGroup: dai.MessageGroup,
    previousVisualization: str,
    title: str = "MessageGroup tree",
) -> Tuple[bool, str, str]:
    visualization = visualizeMessageGroupTree(messageGroup)
    if visualization == previousVisualization:
        return False, previousVisualization, ""

    previousVisualization = visualization
    output = title + ":\n" + visualization
    if not visualization or not visualization.endswith("\n"):
        output += "\n"
    return True, previousVisualization, output
