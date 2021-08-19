import cv2
import matplotlib.pyplot as plt
import numpy as np
import xarray as xr

from backend.py_wake import IEA37SimpleBastankhahGaussian, HorizontalGrid
from backend.py_wake.site import XRSite
from backend.py_wake.site.shear import PowerShear
from backend.py_wake.wind_turbines import WindTurbines
from backend.py_wake.wind_turbines.generic_wind_turbines import GenericWindTurbine, GenericTIRhoWindTurbine


class WindFarm:
    windspeed_upper = 29.6
    windspeed_lower = 0.1

    def __init__(self, wind_speed=windspeed_lower, wind_direction=0):
        self.wts_list = []
        self.wf_model = None
        self.uniform_site = None
        self.wind_speed = wind_speed if self.windspeed_lower < wind_speed < self.windspeed_upper else self.windspeed_lower
        self.wind_direction = wind_direction % 360
        self._x = []
        self._y = []
        self._h = []
        self.flow_box = None
        self.change_happen_since_last_compile = False

    def set_wind_speed(self, ws):
        if ws < self.windspeed_lower:
            self.wind_speed = self.windspeed_lower
        elif ws > self.windspeed_upper:
            self.wind_speed = self.windspeed_upper
        else:
            self.wind_speed = ws

    def set_wind_direction(self, wd):
        self.wind_direction = wd

    def add_wind_turbine(self, name, type_name, diameter, hub_height, xy_pos, power_norm=10000,
                         turbulence_intensity=.1):
        wt = MyWindTurbine2(name, type_name=type_name, diameter=diameter, hub_height=hub_height, xy_pos=xy_pos,
                            power_norm=power_norm,
                            turbulence_intensity=turbulence_intensity)
        self.wts_list.append(wt)
        self.change_happen_since_last_compile = True

    def compile_wind_farm_model(self):
        _wts = []
        self._x = []
        self._y = []
        self._h = []
        for turbine in self.wts_list:
            _wts.append(turbine.wt)
            self._x.append(turbine.x)
            self._y.append(turbine.y)
            self._h.append(turbine.height)
        wts = WindTurbines.from_WindTurbine_lst(_wts)
        turbulence_intensity = .1

        # Site with constant wind speed, sector frequency, constant turbulence intensity and power shear
        # uniform_site = XRSite(
        #     ds=xr.Dataset(data_vars={'WS': self.wind_speed, 'P': ('wd', [1.]), 'TI': turbulence_intensity},
        #                   coords={'wd': [self.wind_direction]}),
        #     shear=PowerShear(h_ref=100, alpha=.2))

        self.uniform_site = XRSite(initial_position=zip(self._x, self._y),
                                   ds=xr.Dataset(data_vars={'P': ('wd', [1.]), 'TI': turbulence_intensity},
                                                 coords={'wd': [self.wind_direction]}),
                                   shear=PowerShear(h_ref=100, alpha=.2))
        self.wf_model = IEA37SimpleBastankhahGaussian(self.uniform_site, wts)
        test_wind = self.uniform_site.local_wind(400, 400, 100)
        self.change_happen_since_last_compile = False

    def generate_flow_map(self, wind_speed=None, wind_direction=None, map_size=800, gen_flow_box=True):
        """
        :param wind_speed: defaults back to class variable
        :param wind_direction: defaults back to class variable
        :param map_size: size of output image.
        :return:
        """
        if wind_speed is None or not (self.windspeed_lower < wind_speed < self.windspeed_upper):
            wind_speed = self.wind_speed
        if wind_direction is None:
            wind_direction = self.wind_direction
        if self.change_happen_since_last_compile:
            self.compile_wind_farm_model()
        sim_res = self.wf_model.__call__(x=self._x, y=self._y, wd=wind_direction, ws=wind_speed, verbose=True)
        ext = 0.12
        flow_map = sim_res.flow_map(grid=HorizontalGrid(resolution=map_size, extend=ext),
                                    # defaults to HorizontalGrid(resolution=500, extend=0.2), see below
                                    wd=wind_direction,
                                    ws=wind_speed)
        dpi = 100
        fig = plt.figure(figsize=(int(map_size / dpi), int(map_size / dpi)), dpi=dpi)
        if gen_flow_box:
            ext = 50
            self.flow_box = sim_res.flow_box(
                x=np.linspace(min(self._x) - ext, max(self._x) + ext, 801),
                y=np.linspace(min(self._y) - ext, max(self._y) + ext, 801),
                h=np.linspace(0, max(self._h) + 50, 25))
        flow_map.plot_wake_map()
        fig.canvas.draw()
        flow_data = flow_map.WS_eff.values.squeeze()
        # flow_data = flow_data * min(255 / 30, 255 / flow_data.max())
        flow_data = flow_data * (255 / flow_data.max())
        flow_data = 255 - flow_data.astype(np.uint8)
        flow_data = np.flip(flow_data, axis=1)
        debug = 0
        return self._convert_to_cv_img(fig), flow_data

    def get_wind_at_pos(self, x, y, z=None):
        if self.flow_box is None:
            print("Need to compute flow box, click the generate flow map button.")
            return False
        res = self.flow_box.WS_eff.sel(x=-x, y=-y, h=-z, method="nearest")
        wd = res.wd.data
        ws = res.values
        debug = 0
        return ws, wd

    @staticmethod
    def _convert_to_cv_img(fig):
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
                            sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img


class MyWindTurbine:
    def __init__(self, name, type_name, diameter, hub_height, xy_pos, power_norm=10000, turbulence_intensity=.1):
        self.name = name
        self.wt = GenericWindTurbine(name=type_name, diameter=diameter, hub_height=hub_height, power_norm=power_norm,
                                     turbulence_intensity=turbulence_intensity)
        self.x = xy_pos[0]
        self.y = xy_pos[1]
        self.height = hub_height


class MyWindTurbine2(MyWindTurbine):
    def __init__(self, name, type_name, diameter, hub_height, xy_pos, power_norm=10000, turbulence_intensity=.1):
        super().__init__(name, type_name, diameter, hub_height, xy_pos, power_norm, turbulence_intensity)
        self.wt = GenericTIRhoWindTurbine(type_name, diameter, hub_height, power_norm=power_norm,
                                          TI_eff_lst=np.linspace(0, .5, 6), default_TI_eff=.1,
                                          Air_density_lst=np.linspace(.9, 1.5, 5), default_Air_density=1.225)


if __name__ == '__main__':
    # turbulence intensity = TI
    test_py_wake = False
    if test_py_wake:
        gen_wt = GenericWindTurbine(name='G10MW', diameter=178.3, hub_height=119, power_norm=10000,
                                    turbulence_intensity=.1)
        wt = GenericTIRhoWindTurbine('2MW', 80, 70, power_norm=2000,
                                     TI_eff_lst=np.linspace(0, .5, 6), default_TI_eff=.1,
                                     Air_density_lst=np.linspace(.9, 1.5, 5), default_Air_density=1.225)

        wts = WindTurbines.from_WindTurbine_lst([gen_wt, gen_wt])
        position = [[1000, 1000], [1500, 1500]]
        # wts.plot([1000, 1500], [1000, 1500])
        # plt.show()

        # Example:
        f = [0.036, 0.039, 0.052, 0.07, 0.084, 0.064, 0.086, 0.118, 0.152, 0.147, 0.1, 0.052]
        wd = np.linspace(0, 360, len(f), endpoint=False)
        ti = .1
        # Site with constant wind speed, sector frequency, constant turbulence intensity and power shear
        uniform_site = XRSite(
            ds=xr.Dataset(data_vars={'WS': 10, 'P': ('wd', f), 'TI': ti},
                          coords={'wd': wd}),
            initial_position=position,
            shear=PowerShear(h_ref=100, alpha=.2))

        wf_model = IEA37SimpleBastankhahGaussian(uniform_site, wts)
        sim_res = wf_model.__call__(x=[1000, 1500], y=[1000, 1500], wd=0, ws=10, verbose=True)
        flow_map = sim_res.flow_map(grid=HorizontalGrid(resolution=800, extend=0.2),
                                    # defaults to HorizontalGrid(resolution=500, extend=0.2), see below
                                    wd=0,
                                    ws=10)
        flow_map.plot_wake_map()
        plt.show()
    else:
        wf = WindFarm(wind_speed=10)
        wf.add_wind_turbine(type_name="SimTurbine", diameter=178.3, hub_height=119, xy_pos=[2000, 1000],
                            power_norm=10000,
                            turbulence_intensity=.1)

        wf.add_wind_turbine(type_name="SimTurbine", diameter=178.3, hub_height=119, xy_pos=[2000, 2000],
                            power_norm=10000,
                            turbulence_intensity=.1)

        wf.add_wind_turbine(type_name="SimTurbine", diameter=178.3, hub_height=119, xy_pos=[3000, 2000],
                            power_norm=10000,
                            turbulence_intensity=.1)
        for i in range(0, 360, 10):
            wf.set_wind_direction(i)
            img = wf.generate_flow_map()
            cv2.imshow("plot", img)
            cv2.waitKey(10)
        cv2.destroyAllWindows()
