"""The browser side, checked without a browser.

There is no JavaScript engine in this test run, so these tests check the things
that break silently in a page and are cheap to check statically: that every
element panel.js reaches for exists in the HTML, that every endpoint it calls
exists in server.py, that nothing is fetched from the network, and that the
design tokens and the stop control are what the design says they are.

A typo in an element id or an endpoint is invisible until the page is open in
front of a running arm, which is the wrong moment to find it.
"""

import re
import unittest
from pathlib import Path

from Christian_control.tools.panel import paths

STATIC = paths.STATIC
HTML = (STATIC / "index.html").read_text()
CSS = (STATIC / "panel.css").read_text()
JS = (STATIC / "panel.js").read_text()
SERVER = (Path(paths.STATIC).parent / "server.py").read_text()

# The approved palette. Colour is reserved for abnormal states, so these are
# the only colours the panel is allowed to have and --ask is the only blue.
TOKENS = {
    "--ground": "#DCDFD8",
    "--panel": "#E8EAE4",
    "--ink": "#14171A",
    "--ink-dim": "#5A625E",
    "--rule": "#A8AEA4",
    "--ask": "#2E6E8E",
    "--warn": "#C46A00",
    "--stop": "#B3161C",
}


def html_ids(text: str) -> set[str]:
    return set(re.findall(r'\bid="([^"]+)"', text))


def js_ids(text: str) -> set[str]:
    return set(re.findall(r"\$\('([^']+)'\)", text))


def js_endpoints(text: str) -> set[str]:
    # Hyphens are part of a route name (/api/controller-log, /api/plan/start-states),
    # so leaving them out of the class silently truncates the endpoint and this
    # guard then checks for a path nobody serves.
    return {match.rstrip("/") for match in re.findall(r"['\"`](/api/[a-z/-]*)", text)}


class StaticFilesPresent(unittest.TestCase):
    def test_the_three_files_the_page_needs_exist(self):
        for name in ("index.html", "panel.css", "panel.js"):
            self.assertTrue((STATIC / name).is_file(), f"{name} is missing")

    def test_the_page_loads_its_own_stylesheet_and_script(self):
        self.assertIn('href="panel.css"', HTML)
        self.assertIn('src="panel.js"', HTML)
        self.assertIn('type="module"', HTML)


class NothingComesFromTheNetwork(unittest.TestCase):
    """The workstation has no network to fetch a font or a library from."""

    def test_no_external_urls_anywhere(self):
        for name, text in (("index.html", HTML), ("panel.css", CSS), ("panel.js", JS)):
            for url in re.findall(r"https?://[^\s\"'()]+", text):
                self.fail(f"{name} refers to {url}; the panel must be self-contained")

    def test_the_only_modules_imported_are_the_panel_s_own(self):
        imported = set(re.findall(r"from\s+'([^']+)'", JS))
        self.assertEqual(imported, {"./scene.js", "./readouts.js"})


class DesignTokens(unittest.TestCase):
    def test_every_token_is_defined_once_with_its_approved_value(self):
        for token, value in TOKENS.items():
            matches = re.findall(rf"{re.escape(token)}:\s*([#A-Za-z0-9]+)", CSS)
            self.assertEqual(matches, [value],
                             f"{token} should be defined exactly once, as {value}")

    def test_numbers_are_tabular(self):
        self.assertIn("font-variant-numeric: tabular-nums", CSS)

    def test_the_font_substitution_is_written_down(self):
        # The design asked for a vendored variable font; system stacks are the
        # honest substitute and the file has to say so.
        self.assertIn("system stacks", CSS)


class TheViewModulesGetTheirTokens(unittest.TestCase):
    """scene.js and readouts.js read colours and fonts from panel.css.

    Each of them reads its variables with a built-in fallback colour, so a
    token this stylesheet forgets to define does not fail loudly — the
    component quietly draws in a colour nobody chose. That is exactly the
    failure the single palette exists to prevent, so it is checked here.
    """

    def read_module(self, name: str) -> str:
        path = STATIC / name
        if not path.is_file():
            self.skipTest(f"{name} has not been written yet")
        # Line comments are dropped first: the modules document their own
        # convention with an illustrative var(--ro-something, fallback).
        return "\n".join(line for line in path.read_text().splitlines()
                         if not line.lstrip().startswith("//"))

    def assert_defined(self, module: str, names: set[str]) -> None:
        for token in sorted(names):
            self.assertRegex(
                CSS, rf"{re.escape(token)}\s*:",
                f"{module} reads {token}, which panel.css does not define")

    def test_scene_tokens_are_defined(self):
        text = self.read_module("scene.js")
        names = set(re.findall(r"getPropertyValue\('(--[a-z-]+)'\)", text))
        self.assertTrue(names, "scene.js reads no tokens; check the pattern")
        self.assert_defined("scene.js", names)

    def test_readout_tokens_are_defined(self):
        text = self.read_module("readouts.js")
        names = set(re.findall(r"var\((--[a-z-]+)", text))
        self.assertTrue(names, "readouts.js reads no tokens; check the pattern")
        self.assert_defined("readouts.js", names)


class WiringMatchesTheMarkup(unittest.TestCase):
    def test_every_element_the_script_reaches_for_exists(self):
        missing = sorted(js_ids(JS) - html_ids(HTML))
        self.assertEqual(missing, [], f"panel.js reads ids that index.html does not define: {missing}")

    def test_every_tab_has_a_view(self):
        views = set(re.findall(r'data-view="([a-z]+)"', HTML))
        ids = html_ids(HTML)
        for view in views:
            self.assertIn(f"view-{view}", ids)

    def test_every_shortcut_points_at_a_real_tab(self):
        views = set(re.findall(r'data-view="([a-z]+)"', HTML))
        for target in re.findall(r'data-goto="([a-z]+)"', HTML):
            self.assertIn(target, views)


class EndpointsMatchTheServer(unittest.TestCase):
    def test_every_endpoint_the_page_calls_is_routed(self):
        for endpoint in sorted(js_endpoints(JS)):
            self.assertIn(f'"{endpoint}"', SERVER,
                          f"panel.js calls {endpoint}, which server.py does not route")


class TheStopControl(unittest.TestCase):
    """The one control whose behaviour is a safety property of this file."""

    def test_the_button_exists_and_says_what_it_does(self):
        self.assertIn('id="stop-button"', HTML)
        self.assertIn("STOP AND RELEASE", HTML)
        # The tooltip has to be plain about there being no ramp.
        self.assertRegex(HTML, r"title=\"[^\"]*no ramp")

    def test_it_is_a_real_button_so_the_keyboard_reaches_it(self):
        self.assertRegex(HTML, r'<button[^>]*id="stop-button"')

    def test_it_is_never_disabled_by_the_script(self):
        # Staleness, a dead stream and a broken view module must not be able to
        # take the stop away.
        self.assertNotRegex(JS, r"stop-button'\)\.disabled")

    def test_it_posts_the_stop_endpoint_directly(self):
        self.assertIn("'/api/session/stop'", JS)


class PermanentSafetyText(unittest.TestCase):
    def test_the_estop_line_is_present_and_not_dismissible(self):
        self.assertIn("Physical E-STOP is not monitored. It is your primary stop.", HTML)
        self.assertNotIn("dismiss", HTML.lower())

    def test_the_hardware_mark_is_permanent(self):
        self.assertIn("HARDWARE", HTML)

    def test_replay_is_called_out_as_not_being_an_arm(self):
        self.assertIn('id="replay-mark"', HTML)


class Accessibility(unittest.TestCase):
    def test_focus_is_visible(self):
        self.assertIn(":focus-visible", CSS)

    def test_reduced_motion_is_respected(self):
        self.assertIn("prefers-reduced-motion", CSS)

    def test_it_collapses_to_one_column(self):
        self.assertIn("max-width: 900px", CSS)

    def test_tabs_are_announced_as_tabs(self):
        self.assertIn('role="tablist"', HTML)
        self.assertIn('role="tabpanel"', HTML)


class NaNIsNeverZero(unittest.TestCase):
    """0.0 in place of a missing value reads as perfect tracking."""

    def test_the_number_parser_returns_null_for_anything_non_finite(self):
        self.assertIn("function numOrNull", JS)
        self.assertIn("Number.isFinite(value) ? value : null", JS)


if __name__ == "__main__":
    unittest.main()
