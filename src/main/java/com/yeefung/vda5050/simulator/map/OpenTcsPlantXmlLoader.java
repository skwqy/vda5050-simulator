package com.yeefung.vda5050.simulator.map;

import com.yeefung.vda5050.simulator.map.PlantPath.ConnectionType;
import com.yeefung.vda5050.simulator.map.PlantPath.LayoutControlPoint;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/** Loads OpenTCS 6.x driving course XML ({@code <model>} with {@code <point>} and {@code <path>}). */
public final class OpenTcsPlantXmlLoader {

  private static final System.Logger LOG = System.getLogger(OpenTcsPlantXmlLoader.class.getName());

  private OpenTcsPlantXmlLoader() {}

  public static PlantModel load(Path xmlFile) throws Exception {
    try (InputStream in = Files.newInputStream(xmlFile)) {
      return load(in, xmlFile.getFileName().toString());
    }
  }

  public static PlantModel load(InputStream in, String debugName) throws Exception {
    DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
    f.setNamespaceAware(false);
    f.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
    Document doc = f.newDocumentBuilder().parse(in);
    Element root = doc.getDocumentElement();
    if (root == null || !"model".equals(root.getTagName())) {
      throw new IllegalArgumentException("Expected root element <model> in " + debugName);
    }
    String modelName = attr(root, "name");
    PlantModel.Builder b = PlantModel.builder(modelName);

    NodeList points = root.getElementsByTagName("point");
    for (int i = 0; i < points.getLength(); i++) {
      if (!(points.item(i) instanceof Element el)) {
        continue;
      }
      if (!"point".equals(el.getTagName())) {
        continue;
      }
      String pname = attr(el, "name");
      if (pname == null || pname.isEmpty()) {
        continue;
      }
      long px = parseLongAttr(el, "positionX", 0L);
      long py = parseLongAttr(el, "positionY", 0L);
      b.putPoint(pname, new PointMm(px, py));
    }

    NodeList paths = root.getElementsByTagName("path");
    for (int i = 0; i < paths.getLength(); i++) {
      if (!(paths.item(i) instanceof Element pathEl)) {
        continue;
      }
      if (!"path".equals(pathEl.getTagName())) {
        continue;
      }
      String pname = attr(pathEl, "name");
      String src = attr(pathEl, "sourcePoint");
      String dst = attr(pathEl, "destinationPoint");
      long lenMm = parseLongAttr(pathEl, "length", 0L);
      if (pname == null || src == null || dst == null) {
        continue;
      }
      Element layoutEl = findPathLayoutElement(pathEl);
      List<LayoutControlPoint> cps =
          layoutEl != null ? parseControlPoints(layoutEl) : List.of();
      ConnectionType ct =
          layoutEl != null
              ? resolveConnectionType(attr(layoutEl, "connectionType"), cps)
              : ConnectionType.DIRECT;
      b.putPath(
          pname,
          new PlantPath(pname, src, dst, lenMm, ct, List.copyOf(cps))
      );
    }

    PlantModel model = b.build();
    LOG.log(
        System.Logger.Level.INFO,
        () -> "Loaded plant model \"" + modelName + "\" from " + debugName
            + ": " + model.pointCount() + " points, " + model.pathCount() + " paths"
    );
    return model;
  }

  /**
   * Collects all {@code controlPoint} elements under {@code pathLayout} (any depth / namespace).
   * OpenTCS exports may use default xmlns; {@link Element#getChildNodes()} only sees direct children
   * and {@code getTagName()} may be prefixed — both caused missing CPs and DIRECT-only motion.
   */
  private static List<LayoutControlPoint> parseControlPoints(Element pathLayout) {
    List<LayoutControlPoint> cps = new ArrayList<>();
    NodeList nl = pathLayout.getElementsByTagNameNS("*", "controlPoint");
    appendControlPointsFromNodeList(cps, nl);
    if (cps.isEmpty()) {
      nl = pathLayout.getElementsByTagName("controlPoint");
      appendControlPointsFromNodeList(cps, nl);
    }
    return cps;
  }

  private static void appendControlPointsFromNodeList(List<LayoutControlPoint> cps, NodeList nl) {
    for (int i = 0; i < nl.getLength(); i++) {
      if (nl.item(i) instanceof Element cp) {
        double cx = parseDoubleAttr(cp, "x", 0);
        double cy = parseDoubleAttr(cp, "y", 0);
        cps.add(new LayoutControlPoint(cx, cy));
      }
    }
  }

  /** First {@code pathLayout} under this {@code path} (any depth), with or without XML namespace. */
  private static Element findPathLayoutElement(Element pathEl) {
    NodeList nl = pathEl.getElementsByTagNameNS("*", "pathLayout");
    if (nl.getLength() > 0 && nl.item(0) instanceof Element e) {
      return e;
    }
    nl = pathEl.getElementsByTagName("pathLayout");
    if (nl.getLength() > 0 && nl.item(0) instanceof Element e) {
      return e;
    }
    return findChildElement(pathEl, "pathLayout");
  }

  private static Element findChildElement(Element parent, String tag) {
    NodeList children = parent.getChildNodes();
    for (int i = 0; i < children.getLength(); i++) {
      Node n = children.item(i);
      if (n instanceof Element e && localName(e).equals(tag)) {
        return e;
      }
    }
    return null;
  }

  private static String localName(Element e) {
    String ln = e.getLocalName();
    if (ln != null && !ln.isEmpty()) {
      return ln;
    }
    String t = e.getTagName();
    int c = t.indexOf(':');
    return c >= 0 ? t.substring(c + 1) : t;
  }

  /**
   * Maps XML {@code connectionType} and infers from control points when missing or inconsistent.
   * Many exports omit the attribute or mark DIRECT while still listing {@code controlPoint}s; in
   * that case we must not use {@link PathMotionFactory}'s DIRECT branch (which ignores control points).
   */
  private static ConnectionType resolveConnectionType(String raw, List<LayoutControlPoint> cps) {
    int n = cps.size();
    if (raw == null || raw.isBlank()) {
      return inferConnectionTypeFromControlPoints(n);
    }
    String u = raw.trim().toUpperCase();
    ConnectionType parsed =
        switch (u) {
          case "POLYPATH", "ELBOW", "SLANTED" -> ConnectionType.POLYPATH;
          case "BEZIER" -> ConnectionType.BEZIER;
          case "DIRECT" -> ConnectionType.DIRECT;
          default -> null;
        };
    if (parsed == null) {
      return inferConnectionTypeFromControlPoints(n);
    }
    if (parsed == ConnectionType.DIRECT && n > 0) {
      return inferConnectionTypeFromControlPoints(n);
    }
    if (parsed == ConnectionType.BEZIER) {
      if (n > 2) {
        return ConnectionType.POLYPATH;
      }
      if (n < 2) {
        return n == 1 ? ConnectionType.POLYPATH : ConnectionType.DIRECT;
      }
    }
    return parsed;
  }

  /** 0 → DIRECT; 1 → POLYPATH; 2 → BEZIER (cubic); 3+ → POLYPATH. */
  private static ConnectionType inferConnectionTypeFromControlPoints(int n) {
    if (n <= 0) {
      return ConnectionType.DIRECT;
    }
    if (n == 1) {
      return ConnectionType.POLYPATH;
    }
    if (n == 2) {
      return ConnectionType.BEZIER;
    }
    return ConnectionType.POLYPATH;
  }

  private static String attr(Element e, String name) {
    if (!e.hasAttribute(name)) {
      return null;
    }
    return e.getAttribute(name);
  }

  private static long parseLongAttr(Element e, String name, long def) {
    String s = attr(e, name);
    if (s == null || s.isEmpty()) {
      return def;
    }
    try {
      return Long.parseLong(s.trim());
    } catch (NumberFormatException ex) {
      return def;
    }
  }

  private static double parseDoubleAttr(Element e, String name, double def) {
    String s = attr(e, name);
    if (s == null || s.isEmpty()) {
      return def;
    }
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException ex) {
      return def;
    }
  }
}
