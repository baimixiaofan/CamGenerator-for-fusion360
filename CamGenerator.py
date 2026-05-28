# -*- coding: utf-8 -*-
"""
CamGenerator - Fusion 360 盘形凸轮自动创建插件
支持多段式参数化运动规律生成和导入轮廓点数据两种模式
"""

import adsk.core
import adsk.fusion
import traceback
import math
import csv
import os

_app = None
_ui = None
_handlers = []

CMD_ID = 'CamGeneratorCmd'
CMD_NAME = '凸轮生成器'
CMD_TOOLTIP = '自动创建盘形凸轮，支持多段式参数化运动规律和导入轮廓点数据'

MAX_SEGMENTS = 10


# ============================================================
# 运动规律数学函数
# ============================================================

def simple_harmonic_rise(theta, beta, h):
    return h / 2.0 * (1.0 - math.cos(math.pi * theta / beta))

def simple_harmonic_return(theta, beta, h):
    return h / 2.0 * (1.0 + math.cos(math.pi * theta / beta))

def constant_velocity_rise(theta, beta, h):
    return h * theta / beta

def constant_velocity_return(theta, beta, h):
    return h * (1.0 - theta / beta)

def modified_trapezoid_rise(theta, beta, h):
    b = beta
    if theta <= b / 4.0:
        s = h * (2.0 * theta / b - math.sin(4.0 * math.pi * theta / b) / (2.0 * math.pi))
    elif theta <= 3.0 * b / 4.0:
        s = h * (4.0 * theta / b - 1.0) / 2.0
    else:
        s = h * (1.0 - 2.0 * (b - theta) / b + math.sin(4.0 * math.pi * (b - theta) / b) / (2.0 * math.pi))
    return s

def modified_trapezoid_return(theta, beta, h):
    b = beta
    if theta <= b / 4.0:
        s = h * (1.0 - 2.0 * theta / b + math.sin(4.0 * math.pi * theta / b) / (2.0 * math.pi))
    elif theta <= 3.0 * b / 4.0:
        s = h * (1.0 - (4.0 * theta / b - 1.0) / 2.0)
    else:
        s = h * (2.0 * (b - theta) / b - math.sin(4.0 * math.pi * (b - theta) / b) / (2.0 * math.pi))
    return s


def get_motion_func(motionLaw, segType):
    """根据运动规律和段类型返回对应的函数"""
    if segType == 'rise':
        if motionLaw == 'constant_velocity':
            return constant_velocity_rise
        elif motionLaw == 'modified_trapezoid':
            return modified_trapezoid_rise
        return simple_harmonic_rise
    else:  # return
        if motionLaw == 'constant_velocity':
            return constant_velocity_return
        elif motionLaw == 'modified_trapezoid':
            return modified_trapezoid_return
        return simple_harmonic_return


# ============================================================
# 多段式轮廓生成
# ============================================================

def generate_multi_segment_profile(baseRadius, segments, motionLaw, numPoints=360):
    """
    多段式凸轮轮廓生成
    segments: [(segType, angle, lift), ...]  segType='dwell'|'rise'|'return'
    返回 [(angle_deg, radius), ...]
    """
    totalAngle = sum(seg[1] for seg in segments)
    if abs(totalAngle - 360.0) > 0.01:
        segDesc = ' + '.join(f'{seg[1]}°' for seg in segments)
        raise ValueError(
            f'所有段角度之和必须等于360°，当前为{totalAngle}°\n({segDesc})'
        )

    rb = baseRadius
    currentR = rb
    points = []
    numActive = sum(1 for seg in segments if seg[1] > 0.001)
    ptsPerSeg = max(20, numPoints // numActive) if numActive > 0 else numPoints
    angleAccum = 0.0

    for segType, segAngle, segLift in segments:
        if segAngle < 0.001:
            continue

        nPts = max(20, int(ptsPerSeg * segAngle / 360.0))

        if segType == 'dwell':
            for i in range(nPts):
                theta = segAngle * i / nPts
                points.append((angleAccum + theta, currentR))
        else:
            func = get_motion_func(motionLaw, segType)
            startR = currentR
            for i in range(nPts):
                theta = segAngle * i / nPts
                delta = func(theta, segAngle, segLift)
                r = startR + delta if segType == 'rise' else startR - delta
                points.append((angleAccum + theta, r))
            currentR = currentR + segLift if segType == 'rise' else currentR - segLift

        angleAccum += segAngle

    return points


# ============================================================
# CSV 导入
# ============================================================

def load_points_from_csv(filepath):
    points = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                angle = float(row[0].strip())
                radius = float(row[1].strip())
                points.append((angle, radius))
            except ValueError:
                continue
    if len(points) < 3:
        raise ValueError('CSV 文件至少需要 3 个有效数据点')
    points.sort(key=lambda p: p[0])
    return points


# ============================================================
# 建模函数
# ============================================================

def create_cam_body(rootComp, profile_points, thickness, shaftHoleDia):
    sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
    sketch.name = 'CamProfile'

    splinePoints = adsk.core.ObjectCollection.create()
    for angle_deg, radius in profile_points:
        angle_rad = math.radians(angle_deg)
        x = radius * math.cos(angle_rad)
        y = radius * math.sin(angle_rad)
        splinePoints.add(adsk.core.Point3D.create(x, y, 0))

    # 闭合轮廓
    first_angle_rad = math.radians(profile_points[0][0])
    first_r = profile_points[0][1]
    splinePoints.add(adsk.core.Point3D.create(
        first_r * math.cos(first_angle_rad),
        first_r * math.sin(first_angle_rad), 0
    ))

    sketch.sketchCurves.sketchFittedSplines.add(splinePoints)

    if shaftHoleDia > 0:
        center = adsk.core.Point3D.create(0, 0, 0)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(center, shaftHoleDia / 2.0)

    profiles = sketch.profiles
    if profiles.count == 0:
        raise RuntimeError('无法创建有效的草图轮廓，请检查参数')

    profile = profiles.item(0)
    extrudes = rootComp.features.extrudeFeatures
    extInput = extrudes.createInput(
        profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    distance = adsk.core.ValueInput.createByReal(thickness)
    extInput.setDistanceExtent(False, distance)
    extrude = extrudes.add(extInput)
    return extrude.bodies.item(0)


def export_step(design, body, filepath):
    exportMgr = design.exportManager
    stepOptions = exportMgr.createSTEPExportOptions(filepath, body)
    exportMgr.execute(stepOptions)


# ============================================================
# UI 辅助
# ============================================================

def update_segment_visibility(paramInputs, count):
    """显示/隐藏段输入控件"""
    for i in range(MAX_SEGMENTS):
        visible = i < count
        for suffix in ['Type', 'Angle', 'Lift']:
            inp = paramInputs.itemById(f'seg{i}{suffix}')
            if inp:
                inp.isVisible = visible

def update_lift_visibility(paramInputs, count):
    """根据段类型显示/隐藏升程输入"""
    for i in range(count):
        typeInput = paramInputs.itemById(f'seg{i}Type')
        liftInput = paramInputs.itemById(f'seg{i}Lift')
        if typeInput and liftInput:
            typeName = typeInput.selectedItem.name if typeInput.selectedItem else ''
            liftInput.isVisible = (typeName != '停')


# ============================================================
# 事件处理类
# ============================================================

class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.isExecutedWhenPrecessing = False
            inputs = cmd.commandInputs

            # === 模式选择 ===
            modeInput = inputs.addDropDownCommandInput(
                'mode', '生成模式',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            modeItems = modeInput.listItems
            modeItems.add('参数化运动规律', True)
            modeItems.add('导入轮廓点数据', False)

            # === 参数化模式组 ===
            paramGroup = inputs.addGroupCommandInput('paramGroup', '运动规律参数')
            paramGroup.isVisible = True
            paramInputs = paramGroup.children

            # 运动规律类型
            lawInput = paramInputs.addDropDownCommandInput(
                'motionLaw', '运动规律',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            lawItems = lawInput.listItems
            lawItems.add('简谐运动', True)
            lawItems.add('等速运动', False)
            lawItems.add('改进梯形', False)

            # 基圆半径
            paramInputs.addFloatSpinnerCommandInput(
                'baseRadius', '基圆半径', 'mm',
                1.0, 100.0, 0.5, 5.5
            )

            # 段数
            paramInputs.addIntegerSpinnerCommandInput(
                'segCount', '运动段数', 1, MAX_SEGMENTS, 1, 2
            )

            # 预创建所有段输入（最多 MAX_SEGMENTS 段）
            for i in range(MAX_SEGMENTS):
                # 默认 2 段: 第0段=升(180°,6mm), 第1段=回(180°,6mm)
                defaultType = '停'
                defaultAngle = 90.0
                defaultLift = 0.0
                if i == 0:
                    defaultType = '升'
                    defaultAngle = 180.0
                    defaultLift = 6.0
                elif i == 1:
                    defaultType = '回'
                    defaultAngle = 180.0
                    defaultLift = 6.0

                # 段类型
                typeInput = paramInputs.addDropDownCommandInput(
                    f'seg{i}Type', f'段{i+1} 类型',
                    adsk.core.DropDownStyles.TextListDropDownStyle
                )
                typeItems = typeInput.listItems
                typeItems.add('停', defaultType == '停')
                typeItems.add('升', defaultType == '升')
                typeItems.add('回', defaultType == '回')

                # 段角度
                paramInputs.addFloatSpinnerCommandInput(
                    f'seg{i}Angle', f'段{i+1} 角度', 'deg',
                    1.0, 360.0, 1.0, defaultAngle
                )

                # 段升程（仅升/回段显示）
                liftInput = paramInputs.addFloatSpinnerCommandInput(
                    f'seg{i}Lift', f'段{i+1} 升程', 'mm',
                    0.1, 100.0, 0.5, defaultLift
                )
                liftInput.isVisible = (defaultType != '停')

                # 隐藏超出默认段数的控件
                if i >= 2:
                    for inp in [typeInput, liftInput]:
                        inp.isVisible = False
                    paramInputs.itemById(f'seg{i}Angle').isVisible = False

            # === 导入模式组 ===
            importGroup = inputs.addGroupCommandInput('importGroup', '导入轮廓数据')
            importGroup.isVisible = False
            importInputs = importGroup.children
            importInputs.addStringValueInput('filePath', 'CSV 文件路径', '')

            # === 通用参数 ===
            inputs.addFloatSpinnerCommandInput(
                'thickness', '凸轮厚度', 'mm',
                1.0, 500.0, 0.5, 10.0
            )
            inputs.addFloatSpinnerCommandInput(
                'shaftHoleDia', '轴孔直径', 'mm',
                0.0, 200.0, 0.5, 10.0
            )

            # 注册事件
            onExecute = ExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)

            onInputChanged = InputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)

        except:
            _ui.messageBox('CommandCreated 错误:\n{}'.format(traceback.format_exc()))


class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            inputs = args.inputs
            changedInput = args.input

            if changedInput.id == 'mode':
                modeIndex = changedInput.selectedItem.index
                paramGroup = inputs.itemById('paramGroup')
                importGroup = inputs.itemById('importGroup')
                if modeIndex == 0:
                    paramGroup.isVisible = True
                    importGroup.isVisible = False
                else:
                    paramGroup.isVisible = False
                    importGroup.isVisible = True

            elif changedInput.id == 'segCount':
                count = changedInput.value
                update_segment_visibility(inputs, count)
                update_lift_visibility(inputs, count)

            elif changedInput.id.startswith('seg') and changedInput.id.endswith('Type'):
                idx = int(changedInput.id[3:-4])
                typeName = changedInput.selectedItem.name
                liftInput = inputs.itemById(f'seg{idx}Lift')
                if liftInput:
                    liftInput.isVisible = (typeName != '停')

        except:
            _ui.messageBox('InputChanged 错误:\n{}'.format(traceback.format_exc()))


class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)
            rootComp = design.rootComponent

            inputs = args.command.commandInputs
            modeInput = inputs.itemById('mode')
            modeIndex = modeInput.selectedItem.index

            thickness = inputs.itemById('thickness').value
            shaftHoleDia = inputs.itemById('shaftHoleDia').value

            if modeIndex == 0:
                # === 参数化多段模式 ===
                paramGroup = inputs.itemById('paramGroup')
                paramInputs = paramGroup.children

                motionLawItem = paramInputs.itemById('motionLaw').selectedItem
                motionLawName = motionLawItem.name
                lawMap = {
                    '简谐运动': 'simple_harmonic',
                    '等速运动': 'constant_velocity',
                    '改进梯形': 'modified_trapezoid'
                }
                motionLaw = lawMap.get(motionLawName, 'simple_harmonic')

                baseRadius = paramInputs.itemById('baseRadius').value
                segCount = paramInputs.itemById('segCount').value

                segments = []
                for i in range(segCount):
                    typeName = paramInputs.itemById(f'seg{i}Type').selectedItem.name
                    segTypeMap = {'停': 'dwell', '升': 'rise', '回': 'return'}
                    segType = segTypeMap.get(typeName, 'dwell')

                    segAngle = math.degrees(paramInputs.itemById(f'seg{i}Angle').value)

                    if segType == 'dwell':
                        segLift = 0.0
                    else:
                        segLift = paramInputs.itemById(f'seg{i}Lift').value

                    segments.append((segType, segAngle, segLift))

                profile_points = generate_multi_segment_profile(
                    baseRadius, segments, motionLaw, numPoints=360
                )
            else:
                # === 导入模式 ===
                importGroup = inputs.itemById('importGroup')
                importInputs = importGroup.children
                filePath = importInputs.itemById('filePath').value
                if not filePath or not os.path.isfile(filePath):
                    _ui.messageBox('请输入有效的 CSV 文件路径')
                    args.isValidResult = False
                    return
                profile_points = load_points_from_csv(filePath)

            body = create_cam_body(rootComp, profile_points, thickness, shaftHoleDia)

            result = _ui.messageBox(
                '凸轮创建成功！\n\n是否导出为 STEP 文件？',
                '导出 STEP',
                adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                adsk.core.MessageBoxIconTypes.QuestionIconType
            )
            if result == adsk.core.DialogResults.DialogYes:
                fileDialog = _ui.createFileDialog()
                fileDialog.title = '保存 STEP 文件'
                fileDialog.filter = 'STEP 文件 (*.step;*.stp)'
                fileDialog.filterIndex = 0
                fileDialog.isMultiSelectEnabled = False
                dialogResult = fileDialog.showSave()
                if dialogResult == adsk.core.DialogResults.DialogOK:
                    stepPath = fileDialog.filename
                    if not stepPath.lower().endswith(('.step', '.stp')):
                        stepPath += '.step'
                    export_step(design, body, stepPath)
                    _ui.messageBox(f'STEP 文件已导出到:\n{stepPath}')

        except ValueError as ve:
            _ui.messageBox('参数错误:\n{}'.format(str(ve)))
            args.isValidResult = False
        except:
            _ui.messageBox('执行错误:\n{}'.format(traceback.format_exc()))
            args.isValidResult = False


# ============================================================
# 插件入口
# ============================================================

def run(context):
    try:
        global _app, _ui
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        existingCmd = _ui.commandDefinitions.itemById(CMD_ID)
        if existingCmd:
            existingCmd.deleteMe()

        cmdDef = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP
        )
        onCreated = CommandCreatedHandler()
        cmdDef.commandCreated.add(onCreated)
        _handlers.append(onCreated)

        workspace = _ui.workspaces.itemById('FusionSolidEnvironment')
        panel = workspace.toolbarPanels.itemById('SolidCreatePanel')
        control = panel.controls.addCommand(cmdDef)
        control.isPromoted = False

    except:
        if _ui:
            _ui.messageBox('插件加载失败:\n{}'.format(traceback.format_exc()))


def stop(context):
    try:
        cmdDef = _ui.commandDefinitions.itemById(CMD_ID)
        if cmdDef:
            cmdDef.deleteMe()
        workspace = _ui.workspaces.itemById('FusionSolidEnvironment')
        panel = workspace.toolbarPanels.itemById('SolidCreatePanel')
        control = panel.controls.itemById(CMD_ID)
        if control:
            control.deleteMe()
    except:
        if _ui:
            _ui.messageBox('插件卸载失败:\n{}'.format(traceback.format_exc()))
