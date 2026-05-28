# -*- coding: utf-8 -*-
"""
CamGenerator - Fusion 360 盘形凸轮自动创建插件
支持参数化运动规律生成和导入轮廓点数据两种模式
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

# 命令 ID
CMD_ID = 'CamGeneratorCmd'
CMD_NAME = '凸轮生成器'
CMD_TOOLTIP = '自动创建盘形凸轮，支持参数化运动规律和导入轮廓点数据'


# ============================================================
# 运动规律数学函数
# ============================================================

def simple_harmonic_rise(theta, beta, h, rb):
    """简谐运动升程: r = rb + h/2 * (1 - cos(pi*theta/beta))"""
    return rb + h / 2.0 * (1.0 - math.cos(math.pi * theta / beta))


def simple_harmonic_return(theta, beta, h, rb):
    """简谐运动回程: r = rb + h/2 * (1 + cos(pi*theta/beta))"""
    return rb + h / 2.0 * (1.0 + math.cos(math.pi * theta / beta))


def constant_velocity_rise(theta, beta, h, rb):
    """等速运动升程: r = rb + h*theta/beta"""
    return rb + h * theta / beta


def constant_velocity_return(theta, beta, h, rb):
    """等速运动回程: r = rb + h*(1 - theta/beta)"""
    return rb + h * (1.0 - theta / beta)


def modified_trapezoid_rise(theta, beta, h, rb):
    """改进梯形运动升程（分段加速度曲线）"""
    # 改进梯形：加速段(0~beta/4) - 等速段(beta/4~3*beta/4) - 减速段(3*beta/4~beta)
    # 使用正弦加速度过渡
    b = beta
    if theta <= b / 4.0:
        # 加速段
        s = h * (2.0 * theta / b - math.sin(4.0 * math.pi * theta / b) / (2.0 * math.pi))
    elif theta <= 3.0 * b / 4.0:
        # 等速段
        s = h * (4.0 * theta / b - 1.0) / 2.0
    else:
        # 减速段
        s = h * (1.0 - 2.0 * (b - theta) / b + math.sin(4.0 * math.pi * (b - theta) / b) / (2.0 * math.pi))
    return rb + s


def modified_trapezoid_return(theta, beta, h, rb):
    """改进梯形运动回程"""
    b = beta
    if theta <= b / 4.0:
        s = h * (1.0 - 2.0 * theta / b + math.sin(4.0 * math.pi * theta / b) / (2.0 * math.pi))
    elif theta <= 3.0 * b / 4.0:
        s = h * (1.0 - (4.0 * theta / b - 1.0) / 2.0)
    else:
        s = h * (2.0 * (b - theta) / b - math.sin(4.0 * math.pi * (b - theta) / b) / (2.0 * math.pi))
    return rb + s


def generate_cam_profile_points(baseRadius, maxLift, innerDwell, riseAngle,
                                  outerDwell, returnAngle, motionLaw, numPoints=360):
    """
    生成凸轮轮廓点（极坐标形式）
    返回 [(angle_deg, radius), ...]
    """
    totalAngle = innerDwell + riseAngle + outerDwell + returnAngle
    if abs(totalAngle - 360.0) > 0.01:
        raise ValueError(
            f'角度之和必须等于360°，当前为{totalAngle}°\n'
            f'近休止{innerDwell}° + 升程{riseAngle}° + 远休止{outerDwell}° + 回程{returnAngle}°'
        )

    rb = baseRadius
    h = maxLift

    # 选择运动规律函数
    if motionLaw == 'simple_harmonic':
        rise_func = simple_harmonic_rise
        return_func = simple_harmonic_return
    elif motionLaw == 'constant_velocity':
        rise_func = constant_velocity_rise
        return_func = constant_velocity_return
    elif motionLaw == 'modified_trapezoid':
        rise_func = modified_trapezoid_rise
        return_func = modified_trapezoid_return
    else:
        rise_func = simple_harmonic_rise
        return_func = simple_harmonic_return

    points = []
    for i in range(numPoints):
        angle = 360.0 * i / numPoints  # 角度（度）

        if angle < innerDwell:
            # 近休止段
            r = rb
        elif angle < innerDwell + riseAngle:
            # 升程段
            theta = angle - innerDwell
            r = rise_func(theta, riseAngle, h, rb)
        elif angle < innerDwell + riseAngle + outerDwell:
            # 远休止段
            r = rb + h
        else:
            # 回程段
            theta = angle - innerDwell - riseAngle - outerDwell
            r = return_func(theta, returnAngle, h, rb)

        points.append((angle, r))

    return points


def load_points_from_csv(filepath):
    """从 CSV 文件加载凸轮轮廓点 [(angle_deg, radius), ...]"""
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

    # 按角度排序
    points.sort(key=lambda p: p[0])
    return points


# ============================================================
# 建模函数
# ============================================================

def create_cam_body(rootComp, profile_points, thickness, shaftHoleDia):
    """
    根据凸轮轮廓点创建 3D 实体
    profile_points: [(angle_deg, radius), ...]
    """
    # 在 XY 平面创建草图
    sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
    sketch.name = 'CamProfile'

    # 将极坐标点转为笛卡尔坐标
    splinePoints = adsk.core.ObjectCollection.create()
    for angle_deg, radius in profile_points:
        angle_rad = math.radians(angle_deg)
        x = radius * math.cos(angle_rad)
        y = radius * math.sin(angle_rad)
        splinePoints.add(adsk.core.Point3D.create(x, y, 0))

    # 添加第一个点以闭合轮廓
    first_angle_rad = math.radians(profile_points[0][0])
    first_r = profile_points[0][1]
    splinePoints.add(adsk.core.Point3D.create(
        first_r * math.cos(first_angle_rad),
        first_r * math.sin(first_angle_rad),
        0
    ))

    # 创建样条曲线
    spline = sketch.sketchCurves.sketchFittedSplines.add(splinePoints)

    # 创建轴孔圆（如果需要）
    if shaftHoleDia > 0:
        center = adsk.core.Point3D.create(0, 0, 0)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(
            center, shaftHoleDia / 2.0
        )

    # 获取轮廓
    profiles = sketch.profiles
    if profiles.count == 0:
        raise RuntimeError('无法创建有效的草图轮廓，请检查参数')

    profile = profiles.item(0)

    # 拉伸
    extrudes = rootComp.features.extrudeFeatures
    extInput = extrudes.createInput(
        profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    distance = adsk.core.ValueInput.createByReal(thickness)
    extInput.setDistanceExtent(False, distance)
    extrude = extrudes.add(extInput)

    return extrude.bodies.item(0)


def export_step(design, body, filepath):
    """导出实体为 STEP 文件"""
    exportMgr = design.exportManager
    stepOptions = exportMgr.createSTEPExportOptions(filepath, body)
    exportMgr.execute(stepOptions)


# ============================================================
# 事件处理类
# ============================================================

class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    """命令创建时设置 UI 控件"""

    def notify(self, args):
        try:
            cmd = args.command
            cmd.isExecutedWhenPrecessing = False
            inputs = cmd.commandInputs

            # === 模式选择 ===
            modeInput = inputs.addDropDownCommandInput(
                'mode', '生成模式',
                adsk.core.DropDownStyles.LimitedDropDownListStyle
            )
            modeItems = modeInput.listItems
            modeItems.add('参数化运动规律', True, '')
            modeItems.add('导入轮廓点数据', False, '')

            # === 参数化模式组 ===
            paramGroup = inputs.addGroupCommandInput('paramGroup', '运动规律参数')
            paramGroup.isVisible = True
            paramInputs = paramGroup.children

            # 运动规律类型
            lawInput = paramInputs.addDropDownCommandInput(
                'motionLaw', '运动规律',
                adsk.core.DropDownStyles.LimitedDropDownListStyle
            )
            lawItems = lawInput.listItems
            lawItems.add('简谐运动', True, 'simple_harmonic')
            lawItems.add('等速运动', False, 'constant_velocity')
            lawItems.add('改进梯形', False, 'modified_trapezoid')

            # 基圆半径
            paramInputs.addFloatSpinnerCommandInput(
                'baseRadius', '基圆半径', 'mm',
                5.0, 500.0, 0.5, 30.0
            )
            # 最大升程
            paramInputs.addFloatSpinnerCommandInput(
                'maxLift', '最大升程', 'mm',
                0.5, 200.0, 0.5, 20.0
            )
            # 近休止角
            paramInputs.addFloatSpinnerCommandInput(
                'innerDwellAngle', '近休止角', 'deg',
                0.0, 180.0, 1.0, 30.0
            )
            # 升程角
            paramInputs.addFloatSpinnerCommandInput(
                'riseAngle', '升程角', 'deg',
                10.0, 350.0, 1.0, 120.0
            )
            # 远休止角
            paramInputs.addFloatSpinnerCommandInput(
                'outerDwellAngle', '远休止角', 'deg',
                0.0, 180.0, 1.0, 60.0
            )
            # 回程角
            paramInputs.addFloatSpinnerCommandInput(
                'returnAngle', '回程角', 'deg',
                10.0, 350.0, 1.0, 120.0
            )

            # === 导入模式组 ===
            importGroup = inputs.addGroupCommandInput('importGroup', '导入轮廓数据')
            importGroup.isVisible = False
            importInputs = importGroup.children

            # 文件路径
            importInputs.addStringValueInput(
                'filePath', 'CSV 文件路径', ''
            )

            # === 通用参数（放在组外）===
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
    """处理输入变化，切换模式时显示/隐藏控件组"""

    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            changedInput = args.input

            if changedInput.id == 'mode':
                modeIndex = changedInput.selectedItem.index
                paramGroup = inputs.itemById('paramGroup')
                importGroup = inputs.itemById('importGroup')

                if modeIndex == 0:
                    # 参数化模式
                    paramGroup.isVisible = True
                    importGroup.isVisible = False
                else:
                    # 导入模式
                    paramGroup.isVisible = False
                    importGroup.isVisible = True

        except:
            _ui.messageBox('InputChanged 错误:\n{}'.format(traceback.format_exc()))


class ExecuteHandler(adsk.core.CommandEventHandler):
    """执行凸轮创建"""

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
                # === 参数化模式 ===
                paramGroup = inputs.itemById('paramGroup')
                paramInputs = paramGroup.children

                motionLawItem = paramInputs.itemById('motionLaw').selectedItem
                motionLaw = motionLawItem.name  # '简谐运动' / '等速运动' / '改进梯形'

                # 映射运动规律名称到内部标识
                lawMap = {
                    '简谐运动': 'simple_harmonic',
                    '等速运动': 'constant_velocity',
                    '改进梯形': 'modified_trapezoid'
                }
                lawKey = lawMap.get(motionLaw, 'simple_harmonic')

                baseRadius = paramInputs.itemById('baseRadius').value
                maxLift = paramInputs.itemById('maxLift').value
                innerDwell = paramInputs.itemById('innerDwellAngle').value
                riseAngle = paramInputs.itemById('riseAngle').value
                outerDwell = paramInputs.itemById('outerDwellAngle').value
                returnAngle = paramInputs.itemById('returnAngle').value

                # 生成轮廓点
                profile_points = generate_cam_profile_points(
                    baseRadius, maxLift, innerDwell, riseAngle,
                    outerDwell, returnAngle, lawKey, numPoints=360
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

            # 创建凸轮实体
            body = create_cam_body(rootComp, profile_points, thickness, shaftHoleDia)

            # 询问是否导出 STEP
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

        # 检查命令是否已存在
        existingCmd = _ui.commandDefinitions.itemById(CMD_ID)
        if existingCmd:
            existingCmd.deleteMe()

        # 创建命令定义
        cmdDef = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP
        )

        # 注册命令创建事件
        onCreated = CommandCreatedHandler()
        cmdDef.commandCreated.add(onCreated)
        _handlers.append(onCreated)

        # 添加到工具栏 - SOLID > 创建面板
        workspace = _ui.workspaces.itemById('FusionSolidEnvironment')
        panel = workspace.toolbarPanels.itemById('SolidCreatePanel')
        control = panel.controls.addCommand(cmdDef)
        control.isPromoted = False

    except:
        if _ui:
            _ui.messageBox('插件加载失败:\n{}'.format(traceback.format_exc()))


def stop(context):
    try:
        # 清理命令
        cmdDef = _ui.commandDefinitions.itemById(CMD_ID)
        if cmdDef:
            cmdDef.deleteMe()

        # 清理工具栏按钮
        workspace = _ui.workspaces.itemById('FusionSolidEnvironment')
        panel = workspace.toolbarPanels.itemById('SolidCreatePanel')
        control = panel.controls.itemById(CMD_ID)
        if control:
            control.deleteMe()

    except:
        if _ui:
            _ui.messageBox('插件卸载失败:\n{}'.format(traceback.format_exc()))
