// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
/*
Copyright (c) 2018, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Copyright 2019 Blue Cheetah Analog Design Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <iostream>
#include <memory>
#include <utility>

#include <cbag/logging/spdlog.h>

#include <cbag/common/point_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/schematic/cellview.h>
#include <cbag/schematic/instance.h>

#include <cbag/oa/read.h>
#include <cbag/oa/util.h>

namespace cbagoa {

// enum conversion methods

cbag::path_style get_path_style(oa::oaPathStyleEnum oa_enum) {
    return static_cast<cbag::path_style>(oa_enum);
}

cbag::text_align get_text_align(oa::oaTextAlignEnum oa_enum) {
    return static_cast<cbag::text_align>(oa_enum);
}

cbag::orientation get_orientation(oa::oaOrientEnum oa_enum) {
    switch (oa_enum) {
    case oa::oacR0:
        return cbag::orientation::R0;
    case oa::oacR90:
        return cbag::orientation::R90;
    case oa::oacR180:
        return cbag::orientation::R180;
    case oa::oacR270:
        return cbag::orientation::R270;
    case oa::oacMY:
        return cbag::orientation::MY;
    case oa::oacMYR90:
        return cbag::orientation::MYR90;
    case oa::oacMX:
        return cbag::orientation::MX;
    case oa::oacMXR90:
        return cbag::orientation::MXR90;
    default:
        throw std::invalid_argument("Unknown OA orientation code.");
    }
}

cbag::font_t get_font(oa::oaFontEnum oa_enum) { return static_cast<cbag::font_t>(oa_enum); }

cbag::text_disp_format get_text_disp_format(oa::oaTextDisplayFormatEnum oa_enum) {
    return static_cast<cbag::text_disp_format>(oa_enum);
}

cbag::term_attr_type get_term_attr_type(oa::oaTermAttrTypeEnum oa_enum) {
    return static_cast<cbag::term_attr_type>(oa_enum);
}

cbag::sig_type get_sig_type(oa::oaSigTypeEnum oa_enum) {
    return static_cast<cbag::sig_type>(oa_enum);
}

cbag::term_type get_term_type(oa::oaTermTypeEnum oa_enum) {
    return static_cast<cbag::term_type>(oa_enum);
}

// Read method for properties

std::pair<std::string, cbag::value_t> read_prop(oa::oaProp *p) {
    oa::oaString tmp_str;
    p->getName(tmp_str);
    std::string key(tmp_str);
    // NOTE: static_cast for down-casting is bad, but openaccess API sucks...
    switch (p->getType()) {
    case oa::oacStringPropType: {
        p->getValue(tmp_str);
        return {std::move(key), std::string(tmp_str)};
    }
    case oa::oacIntPropType: {
        return {std::move(key),
                static_cast<int_fast32_t>(static_cast<oa::oaIntProp *>(p)->getValue())};
    }
    case oa::oacDoublePropType: {
        return {std::move(key),
                static_cast<double_t>(static_cast<oa::oaDoubleProp *>(p)->getValue())};
    }
    case oa::oacTimePropType: {
        return {std::move(key), cbag::time_struct{static_cast<oa::oaTimeProp *>(p)->getValue()}};
    }
    case oa::oacAppPropType: {
        oa::oaByteArray data;
        oa::oaString app_str;
        oa::oaAppProp *app_ptr = static_cast<oa::oaAppProp *>(p);
        app_ptr->getValue(data);
        app_ptr->getAppType(app_str);
        const unsigned char *data_ptr = data.getElements();
        return {std::move(key),
                cbag::binary_t{(const char *)app_str,
                               std::string(data_ptr, data_ptr + data.getNumElements())}};
    }
    case oa::oacBooleanPropType: {
        return {std::move(key), static_cast<bool>(static_cast<oa::oaBooleanProp *>(p)->getValue())};
    }
    default: {
        throw std::invalid_argument(
            fmt::format("Unsupported OA property {} with type: {}, see developer.", key,
                        (const char *)p->getType().getName()));
    }
    }
}

std::pair<std::string, cbag::value_t> read_app_def(oa::oaDesign *dsn, oa::oaAppDef *p) {
    oa::oaString tmp_str;
    p->getName(tmp_str);
    std::string key(tmp_str);
    // NOTE: static_cast for down-casting is bad, but openaccess API sucks...
    switch (p->getType()) {
    case oa::oacIntAppDefType: {
        return {std::move(key), static_cast<int_fast32_t>(
                                    (static_cast<oa::oaIntAppDef<oa::oaDesign> *>(p))->get(dsn))};
    }
    case oa::oacStringAppDefType: {
        (static_cast<oa::oaStringAppDef<oa::oaDesign> *>(p))->get(dsn, tmp_str);
        return {std::move(key), std::string(tmp_str)};
    }
    default: {
        throw std::invalid_argument(
            fmt::format("Unsupported OA AppDef {} with type: {}, see developer.", key,
                        (const char *)p->getType().getName()));
    }
    }
}

bool save_app_def(const std::string &name) {
    return name == "_dbLastSavedCounter" || name == "_dbvCvTimeStamp" || name == "cdbRevision" ||
           name == "cdnSPDesignMajorVersion";
}

// Read methods for shapes

cbag::sch::rectangle read_rect(oa::oaRect *p, std::string &&net) {
    oa::oaBox box;
    p->getBBox(box);
    cbag::sch::rectangle ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), box.left(),
                             box.bottom(), box.right(), box.top());
    return ans;
}

cbag::sch::polygon read_poly(oa::oaPolygon *p, std::string &&net) {
    oa::oaPointArray arr;
    p->getPoints(arr);
    oa::oaUInt4 size = p->getNumPoints();
    cbag::sch::polygon ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), size);
    for (oa::oaUInt4 idx = 0; idx < size; ++idx) {
        ans.points.emplace_back(cbag::point_t{arr[idx].x(), arr[idx].y()});
    }
    return ans;
}

cbag::sch::arc read_arc(oa::oaArc *p, std::string &&net) {
    oa::oaBox box;
    p->getBBox(box);
    cbag::sch::arc ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), p->getStartAngle(),
                       p->getStopAngle(), box.left(), box.bottom(), box.right(), box.top());
    return ans;
}

cbag::sch::donut read_donut(oa::oaDonut *p, std::string &&net) {
    oa::oaPoint pt;
    p->getCenter(pt);
    cbag::sch::donut ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), p->getRadius(),
                         p->getHoleRadius(), pt.x(), pt.y());
    return ans;
}

cbag::sch::ellipse read_ellipse(oa::oaEllipse *p, std::string &&net) {
    oa::oaBox box;
    p->getBBox(box);
    cbag::sch::ellipse ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), box.left(),
                           box.bottom(), box.right(), box.top());
    return ans;
}

cbag::sch::line read_line(oa::oaLine *p, std::string &&net) {
    oa::oaPointArray arr;
    p->getPoints(arr);
    oa::oaUInt4 size = p->getNumPoints();
    cbag::sch::line ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), size);
    for (oa::oaUInt4 idx = 0; idx < size; ++idx) {
        ans.points.emplace_back(cbag::point_t{arr[idx].x(), arr[idx].y()});
    }
    return ans;
}

cbag::sch::path read_path(oa::oaPath *p, std::string &&net) {
    oa::oaPointArray arr;
    p->getPoints(arr);
    oa::oaUInt4 size = p->getNumPoints();
    cbag::sch::path ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), p->getWidth(), size,
                        get_path_style(p->getStyle()), p->getBeginExt(), p->getEndExt());
    for (oa::oaUInt4 idx = 0; idx < size; ++idx) {
        ans.points.emplace_back(cbag::point_t{arr[idx].x(), arr[idx].y()});
    }
    return ans;
}

cbag::sch::text_t read_text(oa::oaText *p, std::string &&net) {
    oa::oaString text;
    p->getText(text);
    oa::oaPoint pt;
    p->getOrigin(pt);
    bool overbar = (p->hasOverbar() != 0);
    bool visible = (p->isVisible() != 0);
    bool drafting = (p->isDrafting() != 0);
    cbag::sch::text_t ans(p->getLayerNum(), p->getPurposeNum(), std::move(net), std::string(text),
                          get_text_align(p->getAlignment()), get_orientation(p->getOrient()),
                          get_font(p->getFont()), p->getHeight(), overbar, visible, drafting,
                          pt.x(), pt.y());
    return ans;
}

cbag::sch::eval_text read_eval_text(oa::oaEvalText *p, std::string &&net) {
    oa::oaString text, eval;
    p->getText(text);
    p->getEvaluatorName(eval);
    oa::oaPoint pt;
    p->getOrigin(pt);
    bool overbar = (p->hasOverbar() != 0);
    bool visible = (p->isVisible() != 0);
    bool drafting = (p->isDrafting() != 0);
    cbag::sch::eval_text ans(
        p->getLayerNum(), p->getPurposeNum(), std::move(net), std::string(text),
        get_text_align(p->getAlignment()), get_orientation(p->getOrient()), get_font(p->getFont()),
        p->getHeight(), overbar, visible, drafting, std::string(eval), pt.x(), pt.y());
    return ans;
}

/** Returns true if the given shape should be included.
 *
 *  The rules are:
 *  1. if a shape has a pin, don't include it (we already added it to the pins).
 *  2. if a shape is an attribute display of a terminal, don't include it (we
 *     already added it).
 *  3. If a shape is a TextOverride, ignore it.
 *  4. otherwise, include it.
 */
bool include_shape(oa::oaShape *p) {
    if (!p->hasPin()) {
        auto shape_type = p->getType();
        if (shape_type == oa::oacTextOverrideType) {
            return false;
        } else if (shape_type == oa::oacAttrDisplayType) {
            auto disp = static_cast<oa::oaAttrDisplay *>(p);
            if (disp->getObject()->isDesign()) {
                auto obj = static_cast<oa::oaDesignObject *>(disp->getObject());
                if (obj->isBlockObject()) {
                    return !((static_cast<oa::oaBlockObject *>(obj))->isTerm());
                } else {
                    return true;
                }
            } else {
                return true;
            }
        } else {
            return true;
        }
    } else {
        return false;
    }
}

cbag::sch::shape_t read_shape(const oa::oaCdbaNS &ns, spdlog::logger &logger, oa::oaShape *p) {
    std::string net;
    if (p->hasNet()) {
        oa::oaString net_name;
        p->getNet()->getName(ns, net_name);
        net = std::string(net_name);
        logger.info("Shape associated with net: {}", net);
    } else {
        logger.info("Shape has no net");
    }

    // NOTE: static_cast for down-casting is bad, but openaccess API sucks...
    switch (p->getType()) {
    case oa::oacRectType:
        return read_rect(static_cast<oa::oaRect *>(p), std::move(net));
    case oa::oacPolygonType:
        return read_poly(static_cast<oa::oaPolygon *>(p), std::move(net));
    case oa::oacArcType:
        return read_arc(static_cast<oa::oaArc *>(p), std::move(net));
    case oa::oacDonutType:
        return read_donut(static_cast<oa::oaDonut *>(p), std::move(net));
    case oa::oacEllipseType:
        return read_ellipse(static_cast<oa::oaEllipse *>(p), std::move(net));
    case oa::oacLineType:
        return read_line(static_cast<oa::oaLine *>(p), std::move(net));
    case oa::oacPathType:
        return read_path(static_cast<oa::oaPath *>(p), std::move(net));
    case oa::oacTextType:
        return read_text(static_cast<oa::oaText *>(p), std::move(net));
    case oa::oacEvalTextType:
        return read_eval_text(static_cast<oa::oaEvalText *>(p), std::move(net));
    case oa::oacAttrDisplayType: {
        logger.info("Got AttrDisplay shape, which we do not support.  Logging debug information:");
        auto disp = static_cast<oa::oaAttrDisplay *>(p);
        if (disp->getObject()->isDesign()) {
            logger.info("Associated object is a design object.");
            auto obj = static_cast<oa::oaDesignObject *>(disp->getObject());
            if (obj->isBlockObject()) {
                logger.info("dsn->getObject() is a block object.");
                if ((static_cast<oa::oaBlockObject *>(obj))->isTerm()) {
                    logger.info("dsn->getObject() is a terminal.");
                } else {
                    logger.info("dsn->getObject() is not a terminal.");
                }
            } else {
                logger.info("dsn->getObject() is not a block object.");
            }
        } else {
            logger.info("Associated object is not a design object.");
        }
        throw std::invalid_argument(
            "We do not support AttrDisplay shapes.  Did you enable property "
            "displays by accident?  Please provide bag.log to developer.");
    }
    default:
        throw std::invalid_argument(fmt::format("Unsupported OA shape type: {}, see developer.",
                                                (const char *)p->getType().getName()));
    }
}

// Read method for references
cbag::transformation get_xform(const oa::oaTransform &xform) {
    return cbag::transformation(xform.xOffset(), xform.yOffset(), get_orientation(xform.orient()));
}

cbag::sch::instance read_instance(const oa::oaCdbaNS &ns, spdlog::logger &logger, oa::oaInst *p,
                                  const std::unordered_set<std::string> &primitive_libs) {
    // read cellview name
    oa::oaString inst_lib_oa, inst_cell_oa, inst_view_oa;
    p->getLibName(ns, inst_lib_oa);
    p->getCellName(ns, inst_cell_oa);
    p->getViewName(ns, inst_view_oa);

    // read transform and bounding box
    oa::oaTransform xform;
    p->getTransform(xform);
    oa::oaBox bbox;
    p->getBBox(bbox);

    // create instance object
    std::string lib_name(inst_lib_oa);
    cbag::sch::instance inst(lib_name, std::string(inst_cell_oa), std::string(inst_view_oa),
                             get_xform(xform), bbox.left(), bbox.bottom(), bbox.right(),
                             bbox.top());
    inst.is_primitive = primitive_libs.find(lib_name) != primitive_libs.end();
    // read instance parameters
    if (p->hasProp()) {
        oa::oaIter<oa::oaProp> prop_iter(p->getProps());
        oa::oaProp *prop_ptr;
        while ((prop_ptr = prop_iter.getNext()) != nullptr) {
            inst.params.insert(read_prop(prop_ptr));
        }
    }

    // read instance connections
    logger.info("Reading connections");
    oa::oaIter<oa::oaInstTerm> iterm_iter(p->getInstTerms(oacInstTermIterNotImplicit));
    oa::oaInstTerm *iterm_ptr;
    oa::oaString term_name_oa, net_name_oa;
    while ((iterm_ptr = iterm_iter.getNext()) != nullptr) {
        // get terminal and net names
        iterm_ptr->getTermName(ns, term_name_oa);
        auto oa_net_ptr = iterm_ptr->getNet();
        if (!oa_net_ptr) {
            throw std::invalid_argument(
                fmt::format("Terminal {} has no net.", (const char *)term_name_oa));
        }
        oa_net_ptr->getName(ns, net_name_oa);
        logger.info("Terminal {} connected to net {}", (const char *)term_name_oa,
                    (const char *)net_name_oa);
        inst.connections.emplace(std::string(term_name_oa), std::string(net_name_oa));
    }

    return inst;
}

std::pair<std::string, std::unique_ptr<cbag::sch::instance>>
read_instance_pair(const oa::oaCdbaNS &ns, spdlog::logger &logger, oa::oaInst *p,
                   const std::unordered_set<std::string> &primitive_libs) {
    oa::oaString inst_name_oa;
    p->getName(ns, inst_name_oa);
    logger.info("Reading instance {}", (const char *)inst_name_oa);
    return {std::string(inst_name_oa),
            std::make_unique<cbag::sch::instance>(read_instance(ns, logger, p, primitive_libs))};
}

// Read method for pin figures

cbag::sch::pin_figure read_pin_figure(const oa::oaCdbaNS &ns, spdlog::logger &logger, oa::oaTerm *t,
                                      oa::oaPinFig *p,
                                      const std::unordered_set<std::string> &primitive_libs) {
    cbag::sig_type stype = get_sig_type(t->getNet()->getSigType());
    cbag::term_type ttype = get_term_type(t->getTermType());
    if (p->isInst()) {
        cbag::sch::instance inst =
            read_instance(ns, logger, static_cast<oa::oaInst *>(p), primitive_libs);

        oa::oaTextDisplayIter disp_iter(oa::oaTextDisplay::getTextDisplays(t));
        auto *disp_ptr = static_cast<oa::oaAttrDisplay *>(disp_iter.getNext());
        if (disp_ptr == nullptr) {
            oa::oaString term_name_oa;
            t->getName(ns, term_name_oa);
            throw std::invalid_argument(fmt::format(
                "Terminal {} has no attr display.  This sometimes happen on corrupted schematics.  "
                "Please delete the pin and create it again. (get the cellview name from log file)",
                (const char *)term_name_oa));
        }
        if (disp_iter.getNext() != nullptr) {
            oa::oaString term_name_oa;
            t->getName(ns, term_name_oa);
            throw std::invalid_argument(fmt::format("Terminal {} has more than one attr display.",
                                                    (const char *)term_name_oa));
        }

        bool overbar = (disp_ptr->hasOverbar() != 0);
        bool visible = (disp_ptr->isVisible() != 0);
        bool drafting = (disp_ptr->isDrafting() != 0);
        std::string net;
        if (disp_ptr->hasNet()) {
            oa::oaString net_name;
            disp_ptr->getNet()->getName(ns, net_name);
            net = std::string(net_name);
        }
        oa::oaPoint pt;
        disp_ptr->getOrigin(pt);
        cbag::sch::term_attr attr(
            get_term_attr_type(
                oa::oaTermAttrType(disp_ptr->getAttribute().getRawValue()).getValue()),
            disp_ptr->getLayerNum(), disp_ptr->getPurposeNum(), std::move(net),
            get_text_align(disp_ptr->getAlignment()), get_orientation(disp_ptr->getOrient()),
            get_font(disp_ptr->getFont()), disp_ptr->getHeight(),
            get_text_disp_format(disp_ptr->getFormat()), overbar, visible, drafting, pt.x(),
            pt.y());

        return {cbag::sch::pin_object(std::move(inst), std::move(attr)), stype, ttype};
    } else if (p->getType() == oa::oacRectType) {
        oa::oaRect *r = static_cast<oa::oaRect *>(p);
        std::string net;
        if (r->hasNet()) {
            oa::oaString tmp;
            r->getNet()->getName(ns, tmp);
            net = std::string(tmp);
        }
        return {read_rect(r, std::move(net)), stype, ttype};
    } else {
        throw std::invalid_argument(
            fmt::format("Unsupported OA pin figure type: {}, see developer.",
                        (const char *)p->getType().getName()));
    }
}

// Read method for terminals

std::pair<std::string, cbag::sch::pin_figure>
read_terminal_single(const oa::oaCdbaNS &ns, spdlog::logger &logger, oa::oaTerm *term,
                     const std::unordered_set<std::string> &primitive_libs) {
    // parse terminal name
    oa::oaString term_name_oa;
    term->getName(ns, term_name_oa);

    // get pin
    oa::oaIter<oa::oaPin> pin_iter(term->getPins());
    oa::oaPin *pin_ptr = pin_iter.getNext();
    if (pin_ptr == nullptr) {
        throw std::invalid_argument(
            fmt::format("Terminal {} has no pins.", (const char *)term_name_oa));
    }
    if (pin_iter.getNext() != nullptr) {
        throw std::invalid_argument(
            fmt::format("Terminal {} has more than one pin.", (const char *)term_name_oa));
    }

    // get pin figure
    oa::oaIter<oa::oaPinFig> fig_iter(pin_ptr->getFigs());
    oa::oaPinFig *fig_ptr = fig_iter.getNext();
    if (fig_ptr == nullptr) {
        throw std::invalid_argument(
            fmt::format("Terminal {} has no figures.", (const char *)term_name_oa));
    }
    if (fig_iter.getNext() != nullptr) {
        throw std::invalid_argument(
            fmt::format("Terminal {} has more than one figures.", (const char *)term_name_oa));
    }

    return {std::string(term_name_oa), read_pin_figure(ns, logger, term, fig_ptr, primitive_libs)};
};

// Read method for schematic/symbol cell view

cbag::sch::cellview read_sch_cellview(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                                      spdlog::logger &logger, const std::string &lib_name,
                                      const std::string &cell_name, const std::string &view_name,
                                      const std::unordered_set<std::string> &primitive_libs) {
    oa::oaDesign *p =
        open_design(ns_native, logger, lib_name, cell_name, view_name, 'r', oa::oacSchematic);

    logger.info("Reading cellview {}__{}({})", lib_name, cell_name, view_name);
    oa::oaBlock *block = p->getTopBlock();
    oa::oaBox bbox;
    block->getBBox(bbox);

    cbag::sch::cellview ans(lib_name, cell_name, view_name, bbox.left(), bbox.bottom(),
                            bbox.right(), bbox.top());

    // read terminals
    logger.info("Reading terminals");
    oa::oaIter<oa::oaTerm> term_iter(block->getTerms());
    oa::oaTerm *term_ptr;
    oa::oaString tmp;
    while ((term_ptr = term_iter.getNext()) != nullptr) {
        term_ptr->getName(ns, tmp);
        ans.terminals.insert(read_terminal_single(ns, logger, term_ptr, primitive_libs));
    }

    // read shapes
    logger.info("Reading shapes");
    oa::oaIter<oa::oaShape> shape_iter(block->getShapes());
    oa::oaShape *shape_ptr;
    while ((shape_ptr = shape_iter.getNext()) != nullptr) {
        logger.info("shape type: {}", (const char *)shape_ptr->getType().getName());
        // skip shapes associated with pins.  We got those already.
        if (include_shape(shape_ptr)) {
            ans.shapes.push_back(read_shape(ns, logger, shape_ptr));
        } else {
            logger.info("Skipping this shape");
        }
    }

    // read instances
    logger.info("Reading instances");
    oa::oaIter<oa::oaInst> inst_iter(block->getInsts());
    oa::oaInst *inst_ptr;
    while ((inst_ptr = inst_iter.getNext()) != nullptr) {
        // skip instances associated with pins.  We got those already.
        if (!inst_ptr->hasPin()) {
            ans.instances.insert(read_instance_pair(ns, logger, inst_ptr, primitive_libs));
        }
    }

    // read properties
    logger.info("Reading properties");
    oa::oaIter<oa::oaProp> prop_iter(p->getProps());
    oa::oaProp *prop_ptr;
    while ((prop_ptr = prop_iter.getNext()) != nullptr) {
        ans.props.insert(read_prop(prop_ptr));
    }
    logger.info("properties end");

    logger.info("Reading AppDefs");
    oa::oaIter<oa::oaAppDef> appdef_iter(p->getAppDefs());
    oa::oaAppDef *appdef_ptr;
    while ((appdef_ptr = appdef_iter.getNext()) != nullptr) {
        auto cur_appdef = read_app_def(p, appdef_ptr);
        // only read the following AppDefs
        if (save_app_def(cur_appdef.first)) {
            ans.app_defs.insert(std::move(cur_appdef));
        }
    }
    logger.info("AppDefs end");
    logger.info("Finish reading schematic/symbol cellview");

    p->close();
    return ans;
}

void print_prop(oa::oaObject *obj) {
    if (obj->hasProp()) {
        oa::oaIter<oa::oaProp> prop_iter(obj->getProps());
        oa::oaProp *p;
        std::cout << "Reading properties" << std::endl;
        while ((p = prop_iter.getNext()) != nullptr) {
            oa::oaString name;
            oa::oaString val;
            p->getName(name);
            p->getValue(val);
            std::cout << fmt::format("Property name = {}, value = {}, type = {}",
                                     (const char *)name, (const char *)val,
                                     (const char *)p->getType().getName())
                      << std::endl;
            if (val == "oaHierProp") {
                std::cout << "Hierarchical properties:" << std::endl;
                print_prop(p);
            } else if (p->getType().getName() == "AppProp") {
                static_cast<oa::oaAppProp *>(p)->getAppType(val);
                std::cout << fmt::format("AppProp type: {}", (const char *)val) << std::endl;
            }
        }
        std::cout << "properties end" << std::endl;
    } else {
        std::cout << "No properties" << std::endl;
    }
}

template <class T> void print_app_def(T *obj) {
    if (obj->hasAppDef()) {
        oa::oaIter<oa::oaAppDef> appdef_iter(obj->getAppDefs());
        oa::oaAppDef *p;
        std::cout << "Reading AppDefs" << std::endl;
        while ((p = appdef_iter.getNext()) != nullptr) {
            oa::oaString name;
            p->getName(name);
            // NOTE: static_cast for down-casting is bad, but openaccess API sucks...
            switch (p->getType()) {
            case oa::oacIntAppDefType: {
                std::cout << fmt::format("AppDef name: {}, AppDef value: {}", (const char *)name,
                                         (static_cast<oa::oaIntAppDef<T> *>(p))->get(obj))
                          << std::endl;
                break;
            }
            case oa::oacStringAppDefType: {
                oa::oaString val;
                (static_cast<oa::oaStringAppDef<T> *>(p))->get(obj, val);
                std::cout << fmt::format("AppDef name: {}, AppDef value: {}", (const char *)name,
                                         (const char *)val)
                          << std::endl;
                break;
            }
            default: {
                throw std::invalid_argument(
                    fmt::format("Unsupported OA AppDef {} with type: {}, see developer.",
                                (const char *)name, (const char *)p->getType().getName()));
            }
            }
            print_prop(p);
        }
    } else {
        std::cout << "No AppDefs." << std::endl;
    }
}

void print_group(oa::oaGroup *p) {
    oa::oaString grp_str;
    p->getName(grp_str);
    std::cout << fmt::format("group name: {}, domain: {}", (const char *)grp_str,
                             (const char *)p->getGroupDomain().getName())
              << std::endl;
    std::cout << fmt::format("group has prop: {}, has appdef: {}", p->hasProp(), p->hasAppDef())
              << std::endl;
    p->getDef()->getName(grp_str);
    std::cout << fmt::format("group def name: {}", (const char *)grp_str) << std::endl;
    oa::oaIter<oa::oaGroupMember> mem_iter(p->getMembers());
    oa::oaGroupMember *mem_ptr;
    while ((mem_ptr = mem_iter.getNext()) != nullptr) {
        std::cout << fmt::format("group object type: {}",
                                 (const char *)mem_ptr->getObject()->getType().getName())
                  << std::endl;
    }
}

void print_dm_data(oa::oaDMData *data) {
    std::cout << fmt::format("Has app def: {}", data->hasAppDef()) << std::endl;
    print_prop(data);
    std::cout << "Reading groups" << std::endl;
    oa::oaIter<oa::oaGroup> grp_iter(data->getGroups());
    oa::oaGroup *grp_ptr;
    while ((grp_ptr = grp_iter.getNext()) != nullptr) {
        print_group(grp_ptr);
    }
    std::cout << "Groups end" << std::endl;

    std::cout << "Reading AppDefs" << std::endl;
    print_app_def(data);

    std::cout << "Reading AppObjects" << std::endl;
    oa::oaIter<oa::oaAppObjectDef> odef_iter(data->getAppObjectDefs());
    oa::oaAppObjectDef *odef_ptr;
    while ((odef_ptr = odef_iter.getNext()) != nullptr) {
        std::cout << "has object def" << std::endl;
    }
    std::cout << "AppObjects end" << std::endl;
}

void print_cv_props(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns, spdlog::logger &logger,
                    const std::string &lib_name, const std::string &cell_name,
                    const std::string &view_name) {
    oa::oaDesign *p = open_design(ns_native, logger, lib_name, cell_name, view_name, 'r',
                                  (view_name == "layout") ? oa::oacMaskLayout : oa::oacSchematic);

    std::cout << fmt::format("Reading cellview {}__{}({})", lib_name, cell_name, view_name)
              << std::endl;

    // read properties
    print_prop(p);

    std::cout << "Reading design AppDefs" << std::endl;
    print_app_def(p);

    /*
    logger->info("Reading design groups");
    oa::oaIter<oa::oaGroup> grp_iter(p->getGroups(oacGroupIterBlockDomain | oacGroupIterModDomain |
                                                  oacGroupIterNoDomain | oacGroupIterOccDomain));
    oa::oaGroup *grp_ptr;
    while ((grp_ptr = grp_iter.getNext()) != nullptr) {
        print_group(grp_ptr);
    }
    logger->info("Groups end");
    */

    oa::oaScalarName lib_name_oa;
    oa::oaScalarName cell_name_oa;
    oa::oaScalarName view_name_oa;
    p->getLibName(lib_name_oa);
    p->getCellName(cell_name_oa);
    p->getViewName(view_name_oa);

    auto oa_lib_ptr = oa::oaLib::find(lib_name_oa);
    oa_lib_ptr->getAccess(oa::oaLibAccess("read"), 5);
    auto oa_cell_ptr = oa::oaCell::find(oa_lib_ptr, cell_name_oa);
    auto oa_cv_ptr = oa::oaCellView::find(oa_lib_ptr, cell_name_oa, view_name_oa);

    std::cout << "oaCell properties:" << std::endl;
    print_prop(oa_cell_ptr);
    std::cout << "oaCell properties end" << std::endl;

    std::cout << "Reading oaCell AppDefs" << std::endl;
    print_app_def(oa_cell_ptr);

    std::cout << "oaCellView properties:" << std::endl;
    print_prop(oa_cv_ptr);
    std::cout << "oaCellView properties end" << std::endl;

    std::cout << "Reading oaCellView AppDefs" << std::endl;
    print_app_def(oa_cell_ptr);

    std::cout << "Reading cell DM data" << std::endl;
    if (oa::oaCellDMData::exists(lib_name_oa, cell_name_oa)) {
        oa::oaCellDMData *data = oa::oaCellDMData::open(lib_name_oa, cell_name_oa, 'r');
        print_dm_data(data);
    } else {
        std::cout << "no cell DM data." << std::endl;
    }
    std::cout << "cell DM data end" << std::endl;

    std::cout << "Reading cellview DM data" << std::endl;
    if (oa::oaCellViewDMData::exists(lib_name_oa, cell_name_oa, view_name_oa)) {
        oa::oaCellViewDMData *cv_data =
            oa::oaCellViewDMData::open(lib_name_oa, cell_name_oa, view_name_oa, 'r');
        print_dm_data(cv_data);
    } else {
        std::cout << "no cellview DM data." << std::endl;
    }
    std::cout << "cellview DM data end" << std::endl;
    std::cout << "Finish reading cellview" << std::endl;

    oa_lib_ptr->releaseAccess();
    p->close();
}

} // namespace cbagoa
